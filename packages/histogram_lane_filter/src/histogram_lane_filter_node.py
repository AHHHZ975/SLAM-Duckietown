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

        ## New For EKF-SLAM
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

        ## back to original code
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
        self.pub_detection_img = rospy.Publisher("~detection_img", Image, queue_size=1)
        self.pub_path_img = rospy.Publisher("~path_img", Image, queue_size=1)

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
        print("yooo")
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

        # Run EKF-SLAM estimate_pose with current detections
        self.mu, self.Sigma, tags = estimate_pose(
            angular_disp, linear_disp, self.mu, self.Sigma, delta_t, self.detections
        )
        self.loginfo(f"EKF Pose: mu={self.mu}, Sigma={self.Sigma}")
        self.loginfo(f"New tags from estimate_pose: {tags}")

        # Clear detections after processing
        self.detections = []

        # Merge new tags into self.tags
        for k, v in tags.items():
            self.tags[k] = v
        self.loginfo(f"All known tags: {self.tags}")

        self.acc_pos.append((self.mu[0], self.mu[1]))

        # Plot the path
        # self.plot_path_opencv(
        #     self.acc_pos, self.Sigma[0, 0], self.Sigma[1, 1], self.tags
        # )

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
        detected_tags = self.detector.detect(
            img_gray,
            estimate_tag_pose=True,
            camera_params=[308, 323, 315, 244],  # set params
            tag_size=0.05,
        )

        if detected_tags:
            self.loginfo(f"Detected {len(detected_tags)} tag(s)")
            self.detections.append((rospy.Time.now().to_nsec(), detected_tags))
        else:
            pass
            # self.loginfo("No tags detected in current image.")

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
        self.visualize_bounding_boxes_opencv(img_gray, bounding_boxes)

        # publish
        self.publishEstimate(img_msg.header.stamp)

    def visualize_bounding_boxes_opencv(self, image: np.ndarray, bounding_boxes: list):
        # `image` should be a 2D (grayscale) or 3D (BGR) NumPy array
        # If `image` is grayscale, convert to BGR so we can draw colored lines and text
        if len(image.shape) == 2:
            img_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            img_bgr = image.copy()

        for bbox in bounding_boxes:
            center, path, tag_id, _ = bbox
            # center and path are in (x, y) coordinates
            # Convert to int if needed
            cx, cy = int(center[0]), int(center[1])

            # Draw center as a red circle
            cv2.circle(img_bgr, (cx, cy), 3, (0, 0, 255), -1)  # BGR: red

            # Draw polygon (bounding box)
            # path is a list of (x,y) coordinates.
            pts = np.array(path, dtype=np.int32)
            cv2.polylines(
                img_bgr, [pts], isClosed=True, color=(255, 0, 0), thickness=2
            )  # BGR: blue

            # Put the tag_id text near the center
            cv2.putText(
                img_bgr,
                str(tag_id),
                (cx + 5, cy - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1,
                cv2.LINE_AA,
            )

        # Now you have img_bgr with bounding boxes drawn
        # If you need to publish this as a ROS Image, use cv_bridge
        img_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        self.pub_detection_img.publish(img_msg)

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

        # Belief image
        belief_img = self.bridge.cv2_to_imgmsg(
            np.array(255 * self.filter.belief).astype("uint8"), "mono8"
        )
        self.pub_belief_img.publish(belief_img)

        # Segments image
        segments_img = self.bridge.cv2_to_imgmsg(self.filter.image_w_dets)
        self.pub_segments_img.publish(segments_img)

        # Projected segments image
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

    # def plot_path_opencv(vertices, sigma_x, sigma_y, tags):
    #     # Create a blank white image
    #     # Adjust size as needed.
    #     img_size = 600
    #     img = np.ones((img_size, img_size, 3), dtype=np.uint8) * 255

    #     # Define a scaling factor and offset to place coordinates nicely on the image
    #     # Adjust these based on the range of your coordinates
    #     scale = 100.0  # For example, 1 unit in world = 100 pixels
    #     offset_x = img_size // 2
    #     offset_y = img_size // 2

    #     def to_img_coords(x, y):
    #         # Transform world coordinates (x,y) into image coordinates (col, row)
    #         # Assuming x to the right, y upwards
    #         # This places (0,0) at the center of the image
    #         return int(offset_x + x * scale), int(offset_y - y * scale)

    #     # Convert vertices to image coordinates
    #     pts = np.array([to_img_coords(x, y) for x, y in vertices], dtype=np.int32)

    #     # Draw the polygon (path)
    #     # Fill it with orange, then draw black outline
    #     cv2.fillPoly(img, [pts], (0, 165, 255))  # BGR for orange: (0,165,255)
    #     cv2.polylines(img, [pts], False, (0, 0, 0), 2)

    #     # Draw covariance ellipse
    #     # OpenCV expects the axis sizes to be half of the full width/height.
    #     ellipse_center = to_img_coords(vertices[-1][0], vertices[-1][1])
    #     ellipse_axes = (int(sigma_x * scale / 2), int(sigma_y * scale / 2))
    #     cv2.ellipse(
    #         img, ellipse_center, ellipse_axes, 0, 0, 360, (0, 0, 255), 2
    #     )  # Red ellipse

    #     # Filter unique tags and average their positions if repeated
    #     unique_tags = {}
    #     for tag_id, tag_val in tags.items():
    #         if tag_id not in unique_tags:
    #             unique_tags[tag_id] = tag_val
    #         else:
    #             # Average the positions if duplicates
    #             unique_tags[tag_id][0] = (unique_tags[tag_id][0] + tag_val[0]) / 2
    #             unique_tags[tag_id][1] = (unique_tags[tag_id][1] + tag_val[1]) / 2

    #     # Draw tags
    #     for t_id, t_val in unique_tags.items():
    #         tx, ty, err, original_id = t_val
    #         pt = to_img_coords(tx, ty)
    #         # Draw a small circle for the tag
    #         cv2.circle(img, pt, 5, (255, 0, 0), -1)  # Blue circle
    #         # Put text for the tag ID
    #         cv2.putText(
    #             img,
    #             str(original_id),
    #             (pt[0] + 5, pt[1] - 5),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             0.5,
    #             (255, 0, 0),
    #             1,
    #             cv2.LINE_AA,
    #         )

    #     # Auto-scale the display if desired by computing min/max coords
    #     # (You can skip this if you've chosen a suitable scale and offset)

    #     # Now `img` holds the visualization.
    #     # You can show it in a window (if running with display):
    #     # cv2.imshow("Path", img)
    #     # cv2.waitKey(1)

    #     # Or convert it to a ROS Image message if needed:
    #     img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
    #     self.pub_path_img.publish(img_msg)

    #     return img


if __name__ == "__main__":
    lane_filter_node = HistogramLaneFilterNode(node_name="histogram_lane_filter_node")
    rospy.spin()
