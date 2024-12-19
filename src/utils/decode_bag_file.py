#!/usr/bin/env python

import os
import rospy
# from duckietown import DTROS
# from std_msgs.msg import String
# from sensor_msgs.msg import CameraInfo
# from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
import rosbag
import cv2
# from cv_bridge import CvBridge
from duckietown_utils import rgb_from_ros

import argparse


def parse_arguments():
    """
    Parse command-line arguments for the robot name, ROS bag location, and output directory.

    Returns:
        args: Parsed arguments.
    """
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


def process_rosbag(args):
    """
    Process the ROS bag file and extract images and wheel encoder data.

    Args:
        args (argparse.Namespace): Parsed command-line arguments.
    """
    # Create the output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)

    # Load the ROS bag
    bag = rosbag.Bag(args.ros_bag_location)

    # Define topics based on the robot name
    ros_image_topic = f"/{args.robot_name}/camera_node/image/compressed"
    ros_left_wheel_topic = f"/{args.robot_name}/left_wheel_encoder_driver_node/tick"
    ros_right_wheel_topic = f"/{args.robot_name}/right_wheel_encoder_driver_node/tick"

    # Initialize variables
    image_count = 0

    # Open the events CSV file for logging
    events_file_path = os.path.join(args.output_dir, "events.csv")
    with open(events_file_path, "w") as file:
        # Iterate through messages in the specified topics
        for topic, msg, t in bag.read_messages(topics=[ros_image_topic, ros_left_wheel_topic, ros_right_wheel_topic]):
            timestamp = t.to_nsec()  # Convert ROS time to nanoseconds

            # Process image topic
            if topic == ros_image_topic:
                cv_img = rgb_from_ros(msg)  # Convert ROS image to OpenCV format

                # Save the image to the output directory
                image_name = f"frame{image_count:06d}.png"
                image_path = os.path.join(args.output_dir, image_name)
                cv2.imwrite(image_path, cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR))

                # Log the event in the CSV file
                file.write(f"{timestamp},image,{image_name}\n")
                image_count += 1

            # Process left wheel encoder topic
            elif topic == ros_left_wheel_topic:
                file.write(f"{timestamp},left_wheel,{msg.data}\n")

            # Process right wheel encoder topic
            elif topic == ros_right_wheel_topic:
                file.write(f"{timestamp},right_wheel,{msg.data}\n")

    # Close the ROS bag
    bag.close()


def main():
    """Main function to handle argument parsing and processing of the ROS bag."""
    args = parse_arguments()

    # Print parsed arguments for user confirmation
    print(f"Robot Name: {args.robot_name}")
    print(f"ROS Bag Location: {args.ros_bag_location}")
    print(f"Output Directory: {args.output_dir}")

    # Process the ROS bag
    process_rosbag(args)


if __name__ == "__main__":
    main()
