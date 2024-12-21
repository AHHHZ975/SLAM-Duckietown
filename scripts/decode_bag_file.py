#!/usr/bin/env python

import os
import rosbag
import cv2
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

os.makedirs(args.output_dir)

# load the bag
bag = rosbag.Bag(args.ros_bag_location)

robot_name = args.robot_name
ros_image_topic = f"/{robot_name}/camera_node/image/compressed"
image_count = 0

ros_left_wheel_topic = f"/{robot_name}/left_wheel_encoder_driver_node/tick"
ros_right_wheel_topic = f"/{robot_name}/right_wheel_encoder_driver_node/tick"

file = open(os.path.join(args.output_dir, "events.csv"), "w")

for topic,msg,t in bag.read_messages(topics = [ros_image_topic, ros_left_wheel_topic, ros_right_wheel_topic]):
    
    timestamp = t.to_nsec()

    # Storing the image frame somewhere in the hard disk

    if topic == ros_image_topic:
        # Save image
        cv_img = rgb_from_ros(msg)
        
        image_name = "frame%06i.png" %image_count
        cv2.imwrite(os.path.join(args.output_dir, image_name), cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR))
        file.write(f"{timestamp},image,{image_name}\n")

        image_count += 1
    elif topic == ros_left_wheel_topic:
        file.write(f"{timestamp},left_wheel,{msg.data}\n")
    elif topic == ros_right_wheel_topic:
        file.write(f"{timestamp},right_wheel,{msg.data}\n")

file.close()
bag.close()

