#!/usr/bin/env python

import os
import rospy
#from duckietown import DTROS
#from std_msgs.msg import String
#from sensor_msgs.msg import CameraInfo
#from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
import rosbag
import cv2
#from cv_bridge import CvBridge
from duckietown_utils import rgb_from_ros

import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description="Process robot name, ROS bag location, and output directory.")
    
    # Add arguments
    parser.add_argument(
        "-r",
        "--robot_name",
        required=True,
        type=str,
        help="Name of the robot."
    )
    parser.add_argument(
        "-b",
        "--ros_bag_location",
        required=True,
        type=str,
        help="Path to the ROS bag file."
    )
    parser.add_argument(
        "-o",
        "--output_dir",
        required=True,
        type=str,
        help="Path to the output directory."
    )
    
    # Parse the arguments
    args = parser.parse_args()
    return args

args = parse_arguments()
print(f"Robot Name: {args.robot_name}")
print(f"ROS Bag Location: {args.ros_bag_location}")
print(f"Output Directory: {args.output_dir}")

# load the bag
bag = rosbag.Bag(args.ros_bag_location)

count = 0
time = []

robot_name = args.robot_name
ros_image_topic = f"/{robot_name}/camera_node/image/compressed"
ros_left_wheel_topic = f"/{robot_name}/left_wheel_encoder_driver_node/tick"
ros_right_wheel_topic = f"/{robot_name}/right_wheel_encoder_driver_node/tick"


for topic,msg, t in bag.read_messages(topics = [ros_image_topic]):
    
    # Converting the messages related to the camera node existing in the bag file into the RGB image format that is compatible with OpenCV's format
    cv_img = rgb_from_ros(msg)
    print(msg)
    print(msg.vel_left)

    # Storing the image frame somewhere in the hard disk
    cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" %count), cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR))
    count +=1

bag.close()

print("yolo")
