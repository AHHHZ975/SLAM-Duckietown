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

print("hello")

import sys

_, ros_bag_dir, ros_topic, output_dir = sys.argv
print(ros_bag_dir, ros_topic, output_dir)

bag = rosbag.Bag(ros_bag_dir)

#bridge = CvBridge()
count = 0

time = []

for topic,msg, t in bag.read_messages(topics = [ros_topic]):
    
    # Converting the messages related to the camera node existing in the bag file into the RGB image format that is compatible with OpenCV's format
    cv_img = rgb_from_ros(msg)

    # Storing the image frame somewhere in the hard disk
    cv2.imwrite(os.path.join(output_dir, "frame%06i.png" %count), cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR))
    count +=1

bag.close()

print("yolo")
