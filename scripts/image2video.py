import cv2
import rosbag
from cv_bridge import CvBridge

# Parameters
video_rostopic = '/usb_cam/image_raw'  # ROS topic for the image
bag_file = '/home/kimkt0408/bagfiles/2025-06-30-08-20-43.bag'  # Path to the bag file
output_video = '/home/kimkt0408/bagfiles/2025-06-30-08-20-43.mp4'  # Output video file
fps = 30  # Frames per second

# Initialize CvBridge
bridge = CvBridge()

# Open the bag file
bag = rosbag.Bag(bag_file, 'r')
  
# Get the first message to get the frame size
for topic, msg, t in bag.read_messages(topics=[video_rostopic]):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    height, width, layers = cv_image.shape
    break


# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # For mp4 output
video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# Write the images to the video file
for topic, msg, t in bag.read_messages(topics=[video_rostopic]):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    video.write(cv_image)
    #print(t)

# Release the VideoWriter object
video.release()

# Close the bag file
bag.close()

print(f"Video saved as {output_video}")

