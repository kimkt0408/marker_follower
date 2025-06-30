#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped

from marker_detector import MarkerDetector

# Controller parameters
desired_distance = 0.5
k_linear = 0.8
k_angular = 2.0
max_linear_speed = 0.05
max_angular_speed = 0.2

bridge = CvBridge()
cmd_pub = None
image_pub = None
tf_buffer = None

# Initialize detector
detector = MarkerDetector()

def image_callback(msg):
    global tf_buffer

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    annotated, detections = detector.detect(cv_image)

    # Publish annotated image
    annotated_msg = bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
    annotated_msg.header = msg.header
    image_pub.publish(annotated_msg)

    if len(detections) == 0:
        rospy.loginfo("No markers detected.")
        stop_robot()
        return

    # Find marker with ID 1
    # TODO: Change this to use the desired marker IDs (multiple IDs should be handled)
    
    det = None
    for d in detections:
        if d["id"] == 1:
            det = d
            break

    if det is None:
        rospy.loginfo("Marker ID 1 not found in this frame.")
        stop_robot()
        return

    tvec = det["tvec"]

    # Create PoseStamped in camera frame
    pose_cam = PoseStamped()
    pose_cam.header = msg.header
    pose_cam.pose.position.x = tvec[0]
    pose_cam.pose.position.y = tvec[1]
    pose_cam.pose.position.z = tvec[2]
    pose_cam.pose.orientation.w = 1.0

    try:
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
            f"Marker {det['id']}: distance={distance:.2f}m angle={np.degrees(angle):.1f}°")
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
    image_pub = rospy.Publisher("/marker_follower/annotated_image", Image, queue_size=10)

    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    rospy.loginfo("marker follower node started.")
    rospy.spin()
