#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped

print(cv2.__name__, cv2.__version__)
# === ArUco Config ===
aruco_dict_type = cv2.aruco.DICT_4X4_50
marker_length = 0.10   # marker side length in meters

# === Controller Params ===
desired_distance = 0.5
k_linear = 0.8
k_angular = 2.0
max_linear_speed = 0.05     # m/s
max_angular_speed = 0.2     # rad/s

# Camera intrinsics
# TODO: Change these values based on your camera calibration
camera_matrix = np.array([[600, 0, 320],
                          [0, 600, 240],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # Assume no distortion for now

bridge = CvBridge()
cmd_pub = None
tf_buffer = None

# Initialize ArUco detector
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
parameters = cv2.aruco.DetectorParameters()

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

def image_callback(msg):
    global tf_buffer

    # Convert ROS Image to OpenCV
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    corners, ids, _ = detector.detectMarkers(cv_image)

    if ids is None:
        print("No markers detected.")
        print("-------------------------------------------------")
        stop_robot()
        return
    else:
        rospy.loginfo(f"Detected {len(ids)} markers.")
        rospy.loginfo(f"Marker IDs: {ids.flatten()}")

    # For simplicity, pick first detected marker
    marker_corners = corners[0]
    marker_id = ids[0][0]

    # Generate 3D object points of the marker
    half_size = marker_length / 2
    obj_points = np.array([
        [-half_size,  half_size, 0],
        [ half_size,  half_size, 0],
        [ half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)

    # Flatten detected corners to shape (4,2)
    img_points = marker_corners.reshape(-1, 2)

    # Solve PnP to get pose
    retval, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        camera_matrix,
        dist_coeffs
    )
    tvec = tvec.flatten()

    # Build pose in camera frame
    pose_cam = PoseStamped()
    pose_cam.header = msg.header
    pose_cam.pose.position.x = tvec[0]
    pose_cam.pose.position.y = tvec[1]
    pose_cam.pose.position.z = tvec[2]
    pose_cam.pose.orientation.w = 1.0

    try:
        # Transform to base_link
        transform = tf_buffer.lookup_transform(
            "base_link",
            pose_cam.header.frame_id,
            rospy.Time(0),
            rospy.Duration(1.0)
        )
        pose_base = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)

        x = pose_base.pose.position.x
        y = pose_base.pose.position.y

        distance = np.sqrt(x**2 + y**2)
        angle = np.arctan2(y, x)

        # Compute control
        linear_error = distance - desired_distance
        vx = k_linear * linear_error
        omega = k_angular * angle

        vx = np.clip(vx, -max_linear_speed, max_linear_speed)
        omega = np.clip(omega, -max_angular_speed, max_angular_speed)

        if distance < desired_distance + 0.05:
            rospy.loginfo("Reached target distance. Stopping.")
            stop_robot()
            return

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = omega
        cmd_pub.publish(cmd)

        rospy.loginfo_throttle(1,
            f"Marker {marker_id}: distance={distance:.2f}m angle={np.degrees(angle):.1f}°")
        rospy.loginfo_throttle(1,
            f"Control: linear={vx:.2f} m/s angular={np.degrees(omega):.1f}°/s")
        print("=================================================")
    except Exception as e:
        rospy.logwarn_throttle(2, f"TF error: {e}")
        stop_robot()

def stop_robot():
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    cmd_pub.publish(cmd)

if __name__ == "__main__":
    rospy.init_node("aruco_follower")

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    rospy.loginfo("ArUco follower node started.")
    rospy.spin()
