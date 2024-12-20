#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import (
    SegmentList,
    LanePose,
    BoolStamped,
    Twist2DStamped,
    FSMState,
    WheelEncoderStamped,
)
from typing import Union
from dt_computer_vision.camera import CameraModel
from dt_computer_vision.camera.homography import Homography, HomographyToolkit
from dt_computer_vision.ground_projection import GroundProjector

import cv2
from solution.lane_filter import LaneFilterHistogram
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import os
import numpy as np
from cv_bridge import CvBridge


class HistogramLaneFilterNode(DTROS):
    """Generates an estimate of the lane pose.

    Creates a `lane_filter` to get estimates on `d` and `phi`, the lateral and heading deviation from the center of the lane.
    It gets the segments extracted by the line_detector as input and output the lane pose estimate.


    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~filter (:obj:`list`): A list of parameters for the lane pose estimation filter
        ~debug (:obj:`bool`): A parameter to enable/disable the publishing of debug topics and images

    Subscribers:
        ~segment_list (:obj:`SegmentList`): The detected line segments from the line detector
        ~(left/right)_wheel_encoder_driver_node/tick (:obj: `WheelEncoderStamped`): Information from the wheel encoders\

    Publishers:
        ~lane_pose (:obj:`LanePose`): The computed lane pose estimate
        ~segments_img (:obj:`Image`): The detected segments
        ~projected_segments_img (:obj:`Image`): The ground projected segments
        ~belief_img (:obj:`Image`): A visualization of the belief histogram as an image

    """

    def __init__(self, node_name):
        super(HistogramLaneFilterNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION
        )

        veh = os.getenv("VEHICLE_NAME")
        self.right_encoder_ticks = 0
        self.left_encoder_ticks = 0
        self.right_encoder_ticks_delta = 0
        self.left_encoder_ticks_delta = 0
        self.last_encoder_stamp = None
        self.camera_info_received = False

        # For EKF-SLAM
        self.mu = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.Sigma = np.eye(3) * 0.1
        self.detections = []
        self.prev_left_tick = None
        self.prev_right_tick = None
        self.prev_time = rospy.Time.now()

        # back to original code
        self._filter = rospy.get_param("~lane_filter_histogram_configuration", None)
        self._debug = rospy.get_param("~debug", False)
        self._predict_freq = rospy.get_param("~predict_frequency", 30.0)

        # Create the filter
        self.filter = LaneFilterHistogram(**self._filter)

        # Subscribers
        self.sub_image = rospy.Subscriber(
            "~image/compressed", CompressedImage, self.cbImage, queue_size=1
        )

        self.sub_camera_info = rospy.Subscriber(
            "~camera_info", CameraInfo, self.cb_camera_info, queue_size=1
        )

        self.sub_encoder_left = rospy.Subscriber(
            "~left_wheel_encoder_driver_node/tick",
            WheelEncoderStamped,
            self.cbProcessLeftEncoder,
            queue_size=1,
        )

        self.sub_encoder_right = rospy.Subscriber(
            "~right_wheel_encoder_driver_node/tick",
            WheelEncoderStamped,
            self.cbProcessRightEncoder,
            queue_size=1,
        )

        # Publishers
        self.pub_lane_pose = rospy.Publisher(
            "~lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        self.pub_belief_img = rospy.Publisher(
            "~belief_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.pub_segments_img = rospy.Publisher(
            "~segments_img", Image, queue_size=1, dt_topic_type=TopicType.DEBUG
        )

        self.pub_projected_segments_img = rospy.Publisher(
            "~projected_segments_img",
            Image,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG,
        )

        # Set up a timer for prediction (if we got encoder data) since that data can come very quickly
        rospy.Timer(rospy.Duration(1 / self._predict_freq), self.cbPredict)

        self.bridge = CvBridge()

    def cb_camera_info(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.log("Received camera info message")
            # create camera object
            _K = np.reshape(msg.K, (3, 3))
            _K[0][2] = _K[0][2]
            _K[1][2] = _K[1][2] - self.filter._crop_top()
            # - update P
            _P = np.reshape(msg.P, (3, 4))
            _P[0][2] = _P[0][2]
            _P[1][2] = _P[1][2] - self.filter._crop_top()
            camera = CameraModel(
                width=msg.width,
                height=msg.height,
                K=_K,
                D=np.reshape(msg.D, (5,)),
                P=_P,
                H=self.load_extrinsics(),
            )

            projector = GroundProjector(camera)
            self.filter.initialize_camera(camera, projector)
            self.loginfo("Camera model initialized")

        self.camera_info_received = True

    def cbProcessLeftEncoder(self, left_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = left_encoder_msg.resolution
            self.filter.initialized = True
        self.left_encoder_ticks_delta = left_encoder_msg.data - self.left_encoder_ticks
        self.last_encoder_stamp = left_encoder_msg.header.stamp

    def cbProcessRightEncoder(self, right_encoder_msg):
        if not self.filter.initialized:
            self.filter.encoder_resolution = right_encoder_msg.resolution
            self.filter.initialized = True
        self.right_encoder_ticks_delta = (
            right_encoder_msg.data - self.right_encoder_ticks
        )
        self.last_encoder_stamp = right_encoder_msg.header.stamp

    def cbPredict(self, event):
        # first let's check if we moved at all, if not abort
        if self.right_encoder_ticks_delta == 0 and self.left_encoder_ticks_delta == 0:
            return

        self.filter.predict(
            self.left_encoder_ticks_delta, self.right_encoder_ticks_delta
        )
        self.left_encoder_ticks += self.left_encoder_ticks_delta
        self.right_encoder_ticks += self.right_encoder_ticks_delta
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0
        

    def cbImage(self, img_msg):
        """Callback to process the segments

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of processed segments

        """
        # Decode from compressed image with OpenCV
        if not self.camera_info_received:
            return

        try:
            image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        except ValueError as e:
            self.logerr(f"Could not decode image: {e}")
            return
        cropped_image = image[self.filter.crop_top :, :, :]
        
        # After decoding the compressed image:
        img_gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        detected_tags = detector.detect(
            img_gray, estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy],  # Use your camera calibration parameters
            tag_size=0.05
        )

        if detected_tags:
            # Store detections along with a timestamp
            self.detections.append((rospy.Time.now().to_nsec(), detected_tags))
        
        # back to OG code
        lines = self.filter.detect_lines(cropped_image)
        segments = self.filter.lines_to_projected_segments(lines)

        # update
        self.filter.update(segments)

        # publish
        self.publishEstimate(img_msg.header.stamp)

    def publishEstimate(self, stamp):

        [d_max, phi_max] = self.filter.getEstimate()

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = True
        lanePose.status = lanePose.NORMAL

        self.pub_lane_pose.publish(lanePose)
        if self._debug:
            self.debugOutput()

    def debugOutput(self):
        """Creates and publishes debug messages"""

        # Create belief image and publish it
        belief_img = self.bridge.cv2_to_imgmsg(
            np.array(255 * self.filter.belief).astype("uint8"), "mono8"
        )
        self.pub_belief_img.publish(belief_img)
        segments_img = self.bridge.cv2_to_imgmsg(self.filter.image_w_dets)
        self.pub_segments_img.publish(segments_img)
        projected_segments_img = self.bridge.cv2_to_imgmsg(
            cv2.cvtColor(self.filter.image_w_segs_rgb, cv2.COLOR_BGR2RGB)
        )
        self.pub_projected_segments_img.publish(projected_segments_img)

    def loginfo(self, s):
        rospy.loginfo("[%s] %s" % (self.node_name, s))

    def load_extrinsics(self) -> Union[Homography, None]:
        """
        Loads the homography matrix from the extrinsic calibration file.

        Returns:
            :obj:`Homography`: the loaded homography matrix

        """
        # load extrinsic calibration
        cali_file_folder = "/data/config/calibrations/camera_extrinsic/"
        cali_file = cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(cali_file):
            self.log(
                f"Can't find calibration file: {cali_file}\n Using default calibration instead.",
                "warn",
            )
            cali_file = os.path.join(cali_file_folder, "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            msg = "Found no calibration file ... aborting"
            self.logerr(msg)
            rospy.signal_shutdown(msg)

        try:
            H: Homography = HomographyToolkit.load_from_disk(
                cali_file, return_date=False
            )  # type: ignore
            return H.reshape((3, 3))
        except Exception as e:
            msg = f"Error in parsing calibration file {cali_file}:\n{e}"
            self.logerr(msg)
            rospy.signal_shutdown(msg)


if __name__ == "__main__":
    lane_filter_node = HistogramLaneFilterNode(node_name="histogram_lane_filter_node")
    rospy.spin()
