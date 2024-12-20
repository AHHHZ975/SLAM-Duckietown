#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import WheelEncoderStamped
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from dt_apriltags import Detector
import os
import numpy as np
from collections import defaultdict


class EKFSLAMNode(DTROS):
    """Extended Kalman Filter SLAM Node for Duckiebot.

    This node performs EKF-based SLAM by using AprilTags detected from the camera and wheel encoder measurements
    to estimate the robot's pose and map static landmarks.

    Args:
        node_name (str): A unique, descriptive name for the node that ROS will use.

    Subscriptions:
        ~camera/image/compressed (sensor_msgs/CompressedImage): Compressed camera images.
        ~camera_info (sensor_msgs/CameraInfo): Camera intrinsic parameters.
        ~left_wheel_encoder_driver_node/tick (duckietown_msgs/WheelEncoderStamped): Left wheel encoder ticks.
        ~right_wheel_encoder_driver_node/tick (duckietown_msgs/WheelEncoderStamped): Right wheel encoder ticks.

    Publications:
        ~pose (geometry_msgs/PoseWithCovarianceStamped): The estimated robot pose.
    """

    def __init__(self, node_name):
        super(EKFSLAMNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # ROS Parameters
        self.robot_name = os.getenv("VEHICLE_NAME", "duckiebot")
        self.camera_info_received = False
        self.bridge = CvBridge()

        # EKF State Variables
        self.mu = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.Sigma = np.eye(3) * 0.1
        self.tags = {}  # Known landmark positions

        # Encoder Variables
        self.left_ticks = None
        self.right_ticks = None
        self.resolution = 135

        # AprilTag Detector
        self.detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        # Subscribers
        self.sub_image = rospy.Subscriber(
            "~camera/image/compressed", Image, self.cb_camera_image, queue_size=1
        )
        self.sub_camera_info = rospy.Subscriber(
            "~camera_info", CameraInfo, self.cb_camera_info, queue_size=1
        )
        self.sub_left_encoder = rospy.Subscriber(
            "~left_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.cb_left_encoder, queue_size=1
        )
        self.sub_right_encoder = rospy.Subscriber(
            "~right_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.cb_right_encoder, queue_size=1
        )

        # Publishers
        self.pub_pose = rospy.Publisher(
            "~pose", PoseWithCovarianceStamped, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

    def cb_camera_info(self, msg):
        """Callback for camera info."""
        if not self.camera_info_received:
            self.camera_matrix = np.reshape(msg.K, (3, 3))
            self.camera_info_received = True
            self.loginfo("Camera info received.")

    def cb_left_encoder(self, msg):
        """Callback for left wheel encoder ticks."""
        self.left_ticks = msg.data

    def cb_right_encoder(self, msg):
        """Callback for right wheel encoder ticks."""
        self.right_ticks = msg.data

    def cb_camera_image(self, msg):
        """Callback for processing camera images."""
        if not self.camera_info_received:
            return

        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg)
            grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detected_tags = self.detector.detect(
                grayscale_image,
                estimate_tag_pose=True,
                camera_params=[self.camera_matrix[0, 0], self.camera_matrix[1, 1], self.camera_matrix[0, 2], self.camera_matrix[1, 2]],
                tag_size=0.05,
            )
            self.update_landmarks(detected_tags)
        except Exception as e:
            self.logerr(f"Failed to process image: {e}")

    def update_landmarks(self, detected_tags):
        """Update landmarks and pose using detected tags."""
        detections = []
        for tag in detected_tags:
            detections.append(
                {"id": tag.tag_id, "pose": tag.pose_t.flatten(), "error": tag.pose_err}
            )

        # Perform EKF update
        if self.left_ticks is not None and self.right_ticks is not None:
            delta_lphi = self.delta_phi(self.left_ticks, self.resolution)
            delta_rphi = self.delta_phi(self.right_ticks, self.resolution)
            angular_disp, linear_disp = self.calculate_displacement(0.0318, 0.1, delta_lphi, delta_rphi)

            self.mu, self.Sigma, updated_tags = self.estimate_pose(
                angular_disp, linear_disp, self.mu, self.Sigma, 0.1, detections
            )
            self.tags.update(updated_tags)
            self.publish_pose()

    def estimate_pose(self, angular_disp, linear_disp, mu, Sigma, delta_t, detections):
        """Extended Kalman Filter logic for SLAM."""
        # EKF prediction and update logic goes here
        # Use your existing `estimate_pose` implementation
        return mu, Sigma, {}

    def delta_phi(self, ticks, resolution):
        """Calculate wheel rotation in radians."""
        return 2 * np.pi * ticks / resolution

    def calculate_displacement(self, R, baseline, delta_lphi, delta_rphi):
        """Calculate linear and angular displacements."""
        linear_disp = R * (delta_lphi + delta_rphi) / 2
        angular_disp = R * (delta_rphi - delta_lphi) / baseline
        return angular_disp, linear_disp

    def publish_pose(self):
        """Publish the estimated pose."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = self.mu[0]
        pose_msg.pose.pose.position.y = self.mu[1]
        pose_msg.pose.pose.orientation.z = self.mu[2]
        pose_msg.pose.covariance[0:3] = self.Sigma.flatten()
        self.pub_pose.publish(pose_msg)


if __name__ == "__main__":
    ekf_slam_node = EKFSLAMNode(node_name="ekf_slam_node")
    rospy.spin()