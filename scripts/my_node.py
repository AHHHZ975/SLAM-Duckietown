#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from duckietown_msgs.msg import Pose2DStamped, WheelsCmdStamped
import rosbag
import cv2
from cv_bridge import CvBridge
from duckietown_utils import rgb_from_ros

print("hello")

bag = rosbag.Bag("/duckietown/my-ros-program/dec_17/luthor_2019-12-17-23-57-40.bag")

bridge = CvBridge()
count = 0

time = []

for topic,msg, t in bag.read_messages(topics = ["/luthor/camera_node/image/compressed"]):
    
    # Converting the messages related to the camera node existing in the bag file into the RGB image format that is compatible with OpenCV's format
    cv_img = rgb_from_ros(msg)

    # Storing the image frame somewhere in the hard disk
    cv2.imwrite(os.path.join("/duckietown/my-ros-program/dec_17/images_luthor_2019-12-17-23-57-40", "frame%06i.png" %count), cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR))
    count +=1

bag.close()

print("yolo")
