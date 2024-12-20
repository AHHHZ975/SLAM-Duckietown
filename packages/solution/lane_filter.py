from collections import OrderedDict
from scipy.stats import multivariate_normal
import numpy as np
from scipy.ndimage.filters import gaussian_filter
from math import floor, sqrt

from solution.histogram_filter import (
    histogram_update,
    histogram_predict,
    histogram_prior,
)
from dt_computer_vision.ground_projection import GroundProjector
from typing import Tuple, Dict, Union, List
from dt_computer_vision.camera import CameraModel, NormalizedImagePoint, Pixel
from dt_computer_vision.ground_projection import GroundProjector
from dt_computer_vision.ground_projection.rendering import draw_grid_image, debug_image
from dt_computer_vision.ground_projection.types import GroundPoint
from dt_computer_vision.line_detection import LineDetector, ColorRange, Detections
from dt_computer_vision.line_detection.rendering import draw_segments
from dt_state_estimation.lane_filter.types import Segment, SegmentColor, SegmentPoint

Color = Tuple[int, int, int]


class LaneFilterHistogram:
    """Generates an estimate of the lane pose.


    Creates and maintain a histogram grid filter to estimate the lane pose.
    Lane pose is defined as the tuple (`d`, `phi`) : lateral deviation and angulare deviation from the
    center of the lane.

    Predict step : Uses the estimated linear and angular velocities to predict the change in the lane pose.
    Update Step : The filter receives a segment list. For each segment, it extracts the corresponding lane
    pose "votes",
    and adds it to the corresponding part of the histogram.

    Best estimate correspond to the slot of the histogram with the highest voted value.

    Args:
        configuration (:obj:`List`): A list of the parameters for the filter

    """

    mean_d_0: float
    mean_phi_0: float
    sigma_d_0: float
    sigma_phi_0: float
    delta_d: float
    delta_phi: float
    d_max: float
    d_min: float
    phi_max: float
    phi_min: float
    cov_v: float
    linewidth_white: float
    linewidth_yellow: float
    lanewidth: float
    min_max: float
    sigma_d_mask: float
    sigma_phi_mask: float
    range_min: float
    range_est: float
    range_max: float

    def __init__(self, **kwargs):
        param_names = [
            "mean_d_0",
            "mean_phi_0",
            "sigma_d_0",
            "sigma_phi_0",
            "delta_d",
            "delta_phi",
            "d_max",
            "d_min",
            "phi_max",
            "phi_min",
            "linewidth_white",
            "linewidth_yellow",
            "lanewidth",
            "sigma_d_mask",
            "sigma_phi_mask",
            "range_min",
            "range_est",
            "range_max",
            "encoder_resolution",
            "wheel_radius",
            "wheel_baseline",
        ]

        for p_name in param_names:
            assert p_name in kwargs, (p_name, param_names, kwargs)
            setattr(self, p_name, kwargs[p_name])

        self.d, self.phi = np.mgrid[
            self.d_min : self.d_max : self.delta_d,
            self.phi_min : self.phi_max : self.delta_phi,
        ]
        self.grid_spec = {
            "d": self.d,
            "phi": self.phi,
            "delta_d": self.delta_d,
            "delta_phi": self.delta_phi,
            "d_min": self.d_min,
            "d_max": self.d_max,
            "phi_min": self.phi_min,
            "phi_max": self.phi_max,
            "range_min": self.range_min,
            "range_est": self.range_est,
            "range_max": self.range_max,
        }
        self.road_spec = {
            "linewidth_white": self.linewidth_white,
            "linewidth_yellow": self.linewidth_yellow,
            "lanewidth": self.lanewidth,
        }
        self.robot_spec = {
            "wheel_radius": self.wheel_radius,
            "wheel_baseline": self.wheel_baseline,
            "encoder_resolution": self.encoder_resolution,
        }
        self.belief = np.empty(self.d.shape)
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0 = [[self.sigma_d_0, 0], [0, self.sigma_phi_0]]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]

        # Additional variables
        self.initialized = False
        self.camera_initialized = False
        self.crop_top = 200
        self.initialize()
        # colors
        self.color_ranges: Dict[str, ColorRange] = {
            "white": ColorRange.fromDict({"low": [0, 0, 150], "high": [180, 100, 255]}),
            "yellow": ColorRange.fromDict(
                {"low": [0, 100, 100], "high": [45, 255, 255]}
            ),
        }
        self.colors: Dict[str, Color] = {
            "red": (0, 0, 255),
            "yellow": (0, 255, 255),
            "white": (255, 255, 255),
        }
        self.color_order = ["yellow", "white"]
        self.colors_to_detect = [self.color_ranges[c] for c in self.color_order]

    def _crop_top(self):
        return self.crop_top

    def detect_lines(self, img_cropped):
        if not self.camera_initialized:
            return
        detector = LineDetector()
        color_detections: List[Detections] = detector.detect(
            img_cropped, self.colors_to_detect
        )
        lines: Dict[str, dict] = {}
        for i, detections in enumerate(color_detections):
            color = self.color_order[i]
            # pack detections in a dictionary
            lines[color] = {
                "lines": detections.lines.tolist(),
                "centers": detections.centers.tolist(),
                "normals": detections.normals.tolist(),
                "color": self.color_ranges[color].representative,
            }
        image_w_dets = draw_segments(
            img_cropped, {self.color_ranges["yellow"]: color_detections[0]}
        )
        self.image_w_dets = draw_segments(
            image_w_dets, {self.color_ranges["white"]: color_detections[1]}
        )
        return lines

    def lines_to_projected_segments(self, lines):
        segments: List[Segment] = []
        colored_segments: Dict[Color, List[Tuple[GroundPoint, GroundPoint]]] = {}
        grid = draw_grid_image((400, 400))

        for color, colored_lines in lines.items():
            grounded_segments: List[Tuple[GroundPoint, GroundPoint]] = []
            for line in colored_lines["lines"]:
                # distorted pixels
                p0: Pixel = Pixel(line[0], line[1])
                p1: Pixel = Pixel(line[2], line[3])
                # distorted pixels to rectified pixels
                p0_rect: Pixel = self.camera.rectifier.rectify_pixel(p0)
                p1_rect: Pixel = self.camera.rectifier.rectify_pixel(p1)
                # rectified pixel to normalized coordinates
                p0_norm: NormalizedImagePoint = self.camera.pixel2vector(p0_rect)
                p1_norm: NormalizedImagePoint = self.camera.pixel2vector(p1_rect)
                # project image point onto the ground plane
                grounded_p0: SegmentPoint = self.projector.vector2ground(p0_norm)
                grounded_p1: SegmentPoint = self.projector.vector2ground(p1_norm)
                # add grounded segment to output
                segments.append(
                    Segment(
                        points=[grounded_p0, grounded_p1], color=SegmentColor(color)
                    )
                )
                grounded_segments.append((grounded_p0, grounded_p1))

            colored_segments[self.colors[color]] = grounded_segments
        image_w_segs = debug_image(colored_segments, (400, 400), background_image=grid)
        self.image_w_segs_rgb = image_w_segs[:, :, [2, 1, 0]]
        return segments

    def initialize(self):
        self.belief = histogram_prior(
            self.belief, self.grid_spec, self.mean_0, self.cov_0
        )
        print("Histogram Filter Initialized")
        self.initialized = True

    def initialize_camera(self, camera, projector):
        self.camera = camera
        self.projector = projector
        self.camera_initialized = True

    def predict(self, left_encoder_delta_ticks, right_encoder_delta_ticks):
        if not self.initialized:
            return
        self.belief = histogram_predict(
            self.belief,
            left_encoder_delta_ticks,
            right_encoder_delta_ticks,
            self.grid_spec,
            self.robot_spec,
            self.cov_mask,
        )

    def update(self, segments):
        if not self.initialized:
            return
        (measurement_likelihood, self.belief) = histogram_update(
            self.belief, segments, self.road_spec, self.grid_spec
        )

    def getEstimate(self):
        maxids = np.unravel_index(self.belief.argmax(), self.belief.shape)
        d_max = self.d_min + (maxids[0] + 0.5) * self.delta_d
        phi_max = self.phi_min + (maxids[1] + 0.5) * self.delta_phi

        return [d_max, phi_max]
