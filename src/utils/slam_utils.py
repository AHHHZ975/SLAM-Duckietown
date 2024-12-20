from typing import Tuple, List
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Ellipse, PathPatch
from matplotlib import patches
from matplotlib.path import Path
import cv2
import argparse
import os
from collections import defaultdict
from dt_apriltags import Detector

# Constants
UNIQUE_COLOR = [
    "#e6194B",
    "#3cb44b",
    "#ffe119",
    "#4363d8",
    "#f58231",
    "#911eb4",
    "#42d4f4",
    "#f032e6",
    "#bfef45",
    "#fabed4",
    "#469990",
    "#dcbeff",
    "#9A6324",
    "#fffac8",
    "#800000",
    "#aaffc3",
    "#808000",
    "#ffd8b1",
    "#000075",
    "#a9a9a9",
]
DELTA_TIME = 500_000_000  # 2 seconds
TAG_TO_INDEX = {}


def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Process decoded directory.")
    parser.add_argument(
        "-d", "--dir", required=True, type=str, help="Decoded directory"
    )
    return parser.parse_args()


def get_color(tag_id: int) -> str:
    """Get a unique color for a tag ID."""
    return UNIQUE_COLOR[tag_id % len(UNIQUE_COLOR)]


def load_grayscale(image_path: str) -> np.ndarray:
    """Load a grayscale image."""
    grayscale_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    return grayscale_image.astype(np.uint8)


def detect_tags(img: np.ndarray) -> list:
    """Detect AprilTags in an image."""
    return detector.detect(
        img, estimate_tag_pose=True, camera_params=[340, 336, 328, 257], tag_size=0.05
    )


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> float:
    """Calculate wheel rotation in radians."""
    delta_ticks = ticks - prev_ticks
    alpha = 2 * np.pi / resolution
    return delta_ticks * alpha


def displacement(
    R: float, baseline: float, delta_phi_left: float, delta_phi_right: float
) -> Tuple[float, float]:
    """Calculate angular and linear displacement."""
    linear_displacement = R * (delta_phi_left + delta_phi_right) / 2
    angular_displacement = R * (delta_phi_right - delta_phi_left) / baseline
    return angular_displacement, linear_displacement