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

import matplotlib

matplotlib.use("Agg")  # Important for headless mode
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Ellipse

import cv2
import os
import numpy as np
from cv_bridge import CvBridge

from solution.lane_filter import LaneFilterHistogram
from solution.ekf_slam import (
    delta_phi,
    displacement,
    estimate_pose,
    TAG_TO_INDEX,
    get_color,
)

from dt_apriltags import Detector
from collections import defaultdict

from solution.ekf_slam import *


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

        self.left_encoder_ticks = 0
        self.right_encoder_ticks = 0
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0

        # Keep track of the robot trajectory
        self.acc_pos = [(0.0, 0.0)]
        self.tags = {}

        # AprilTag Detector (if needed here or in ekf_slam)
        self.detector = Detector(
            searchpath=["apriltags"],
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        # back to original code
        self._filter = rospy.get_param("~lane_filter_histogram_configuration", None)
        self._debug = rospy.get_param("~debug", False)
        self._predict_freq = rospy.get_param("~predict_frequency", 30.0)

        # Create the filter
        self.filter = LaneFilterHistogram(**self._filter)

        ## Create figures for visualization
        self.fig_img, self.ax_img = plt.subplots()
        self.fig_path, self.ax_path = plt.subplots()

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
        self.pub_detection_img = rospy.Publisher("~detection_img", Image, queue_size=1)
        self.pub_path_img = rospy.Publisher("~path_img", Image, queue_size=1)

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

        # Set up a timer for prediction (if we got encoder data) since that data can come very quickly
        rospy.Timer(rospy.Duration(1 / self._predict_freq), self.cbPredict)

        self.bridge = CvBridge()

        # Create figure and axis for plotting the path
        self.fig_path, self.ax_path = plt.subplots(figsize=(4, 4))
        # Publisher for the path image
        self.pub_path_img = rospy.Publisher("~path_img", Image, queue_size=1)

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
        if self.right_encoder_ticks_delta == 0 and self.left_encoder_ticks_delta == 0:
            return

        # Predict step in lane filter
        self.filter.predict(
            self.left_encoder_ticks_delta, self.right_encoder_ticks_delta
        )

        # Update ticks
        self.left_encoder_ticks += self.left_encoder_ticks_delta
        self.right_encoder_ticks += self.right_encoder_ticks_delta

        # Compute angular and linear displacement
        R = 0.0318  # Example wheel radius
        baseline = 0.1  # Example baseline
        resolution = self.filter.encoder_resolution

        delta_lphi = delta_phi(self.left_encoder_ticks_delta, 0, resolution)
        delta_rphi = delta_phi(self.right_encoder_ticks_delta, 0, resolution)

        angular_disp, linear_disp = displacement(R, baseline, delta_lphi, delta_rphi)

        # Reset deltas
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0

        current_time = rospy.Time.now()
        delta_t = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        # Run estimate_pose with current detections
        self.mu, self.Sigma, tags = estimate_pose(
            angular_disp, linear_disp, self.mu, self.Sigma, delta_t, self.detections
        )

        # Clear detections after processing
        self.detections = []

        # Merge new tags into self.tags
        for k, v in tags.items():
            self.tags[k] = v

        self.acc_pos.append((self.mu[0], self.mu[1]))

        # Plot the path
        self.plot_path(self.acc_pos, self.Sigma[0, 0], self.Sigma[1, 1], self.tags)
        self.publish_path_image()

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
        # Detect apriltags
        img_gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)

        # Set your camera parameters as per your environment
        detected_tags = detector.detect(
            img_gray,
            estimate_tag_pose=True,
            camera_params=[308, 323, 315, 244],  # set params
            tag_size=0.05,
        )

        if detected_tags:
            # Store detections along with a timestamp
            self.detections.append((rospy.Time.now().to_nsec(), detected_tags))

        # Also do line detection and lane filter update as before
        lines = self.filter.detect_lines(cropped_image)
        segments = self.filter.lines_to_projected_segments(lines)
        self.filter.update(segments)

        # Visualize detections
        bounding_boxes = [
            (x.center.tolist(), x.corners.tolist(), x.tag_id, x.pose_t)
            for x in detected_tags
        ]

        # Plot detections on the grayscale image
        self.visualize_bounding_boxes(img_gray, bounding_boxes)
        self.publish_detection_image()

        # publish
        self.publishEstimate(img_msg.header.stamp)

    def visualize_bounding_boxes(self, image: np.ndarray, bounding_boxes: list):
        self.ax_img.clear()
        self.ax_img.imshow(image, cmap="gray")

        for bbox in bounding_boxes:
            center, path, tag_id, _ = bbox
            self.ax_img.plot(center[0], center[1], "ro")
            polygon = Polygon(
                path, closed=True, edgecolor="blue", facecolor="none", lw=2
            )
            self.ax_img.add_patch(polygon)
            self.ax_img.text(
                center[0], center[1], str(tag_id), color=get_color(tag_id), fontsize=10
            )

        self.fig_img.canvas.draw()

    def plot_path(self, vertices, sigma_x, sigma_y, tags):
        self.ax_path.clear()
        x_vals = [v[0] for v in vertices]
        y_vals = [v[1] for v in vertices]
        self.ax_path.plot(x_vals, y_vals, "-o", color="orange")

        # Covariance ellipse
        ellipse = Ellipse(
            (vertices[-1][0], vertices[-1][1]),
            width=sigma_x,
            height=sigma_y,
            edgecolor="red",
            facecolor="none",
        )
        self.ax_path.add_patch(ellipse)

        for t_id, t_pos in tags.items():
            self.ax_path.plot(t_pos[0], t_pos[1], "x", color="blue")
            self.ax_path.text(t_pos[0], t_pos[1], f"Tag:{t_id}")

        self.ax_path.set_aspect("equal", adjustable="box")
        self.ax_path.set_xlabel("X position")
        self.ax_path.set_ylabel("Y position")
        self.ax_path.set_title("Robot Trajectory and Detected Tags")
        self.fig_path.canvas.draw()

    def publish_detection_image(self):
        w, h = self.fig_img.canvas.get_width_height()
        buf = np.frombuffer(self.fig_img.canvas.tostring_rgb(), dtype=np.uint8)
        buf = buf.reshape(h, w, 3)
        img_msg = self.bridge.cv2_to_imgmsg(buf, encoding="rgb8")
        self.pub_detection_img.publish(img_msg)

    def publish_path_image(self):
        w, h = self.fig_path.canvas.get_width_height()
        buf = np.frombuffer(self.fig_path.canvas.tostring_rgb(), dtype=np.uint8)
        buf = buf.reshape(h, w, 3)
        img_msg = self.bridge.cv2_to_imgmsg(buf, encoding="rgb8")
        self.pub_path_img.publish(img_msg)

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

    def plot_path(self, vertices, sigma_x, sigma_y, tags):
        # Clear the axis
        self.ax_path.clear()

        # Plot path
        x_vals = [v[0] for v in vertices]
        y_vals = [v[1] for v in vertices]
        self.ax_path.plot(x_vals, y_vals, "-o", color="orange")

        # Plot covariance ellipse at the last pose
        from matplotlib.patches import Ellipse

        ellipse = Ellipse(
            (vertices[-1][0], vertices[-1][1]),
            width=sigma_x,
            height=sigma_y,
            edgecolor="red",
            facecolor="none",
        )
        self.ax_path.add_patch(ellipse)

        # If you have tag detections, plot them as well
        for t_id, t_pos in tags.items():
            self.ax_path.plot(t_pos[0], t_pos[1], "x", color="blue")
            self.ax_path.text(t_pos[0], t_pos[1], f"Tag:{t_id}")

        # Set equal aspect and some nice bounds
        self.ax_path.set_aspect("equal", adjustable="box")
        self.ax_path.set_xlabel("X position")
        self.ax_path.set_ylabel("Y position")
        self.ax_path.set_title("Robot Trajectory and Detected Tags")

        # Draw the figure in memory
        self.fig_path.canvas.draw()

    def publish_path_image(self):
        # Convert the matplotlib figure to a numpy array
        # The figure is rendered as RGBA
        w, h = self.fig_path.canvas.get_width_height()
        buf = np.frombuffer(self.fig_path.canvas.tostring_rgb(), dtype=np.uint8)
        buf = buf.reshape(h, w, 3)

        # Convert to ROS Image
        img_msg = self.bridge.cv2_to_imgmsg(buf, encoding="rgb8")

        # Publish
        self.pub_path_img.publish(img_msg)


if __name__ == "__main__":
    lane_filter_node = HistogramLaneFilterNode(node_name="histogram_lane_filter_node")
    rospy.spin()
