from typing import Tuple, List
import numpy as np
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


def estimate_pose(
    angular_disp: float,
    linear_disp: float,
    mu: np.ndarray,
    Sigma: np.ndarray,
    delta_t: float,
    detections: list,
) -> Tuple[np.ndarray, np.ndarray, dict]:
    """Estimate robot pose and update belief."""
    detected_tags = defaultdict(list)
    print('posay')
    for timestemp, detected_image in detections:
        for tag in detected_image:
            if tag.tag_id not in TAG_TO_INDEX:
                TAG_TO_INDEX[tag.tag_id] = len(TAG_TO_INDEX)
            detected_tags[TAG_TO_INDEX[tag.tag_id]].append(
                [tag.pose_R, tag.pose_t, tag.pose_err]
            )

    tags = {}
    INV_TAG_TO_INDEX = {v: k for k, v in TAG_TO_INDEX.items()}
    for tag_id, tag_data in detected_tags.items():
        # Check for conflicting data
        if len(tag_data) > 1:
            print(f"Resolving ambiguity for tag {tag_id}")

        # Aggregate tag data
        t = np.mean([tag[1] for tag in tag_data], axis=0)
        err = np.mean([tag[2] for tag in tag_data], axis=0)
        x, z = t[0], t[2]

        # Transform tag position relative to robot
        rel_x = np.cos(mu[2]) * z + np.sin(mu[2]) * x
        rel_y = np.sin(mu[2]) * z - np.cos(mu[2]) * x

        # Save resolved tag
        tags[tag_id] = [mu[0] + rel_x, mu[1] + rel_y, err, INV_TAG_TO_INDEX[tag_id]]

    inter_vw = linear_disp / angular_disp if angular_disp != 0 else linear_disp
    if angular_disp == 0:
        mu[0] += inter_vw * np.cos(mu[2])
        mu[1] += inter_vw * np.sin(mu[2])
        G = np.array(
            [
                [1, 0, -inter_vw * np.sin(mu[2])],
                [0, 1, inter_vw * np.cos(mu[2])],
                [0, 0, 1],
            ]
        )
    else:
        mu[0] += inter_vw * (np.sin(mu[2] + angular_disp) - np.sin(mu[2]))
        mu[1] += inter_vw * (np.cos(mu[2]) - np.cos(mu[2] + angular_disp))
        mu[2] += angular_disp
        G = np.array(
            [
                [1, 0, inter_vw * (np.cos(mu[2]) - np.cos(mu[2] + angular_disp))],
                [0, 1, inter_vw * (np.sin(mu[2] + angular_disp) - np.sin(mu[2]))],
                [0, 0, 1],
            ]
        )

    Sigma = G @ Sigma @ G.T + np.eye(3) * 0.05 * delta_t
    print("Detected tags:", detected_tags)
    print("Pose estimate:", mu, Sigma)
    return mu, Sigma, tags


def plot_path(
    vertices: List[Tuple[float, float]], sigma_x: float, sigma_y: float, tags: dict
):
    """Plot the robot path and detected tags."""
    ax_path.cla()  # Clear the current axis
    path = Path(vertices)
    patch = PathPatch(path, facecolor="orange", edgecolor="black", lw=2)
    ax_path.add_patch(patch)

    # Covariance ellipse
    ellipse = Ellipse(
        (vertices[-1][0], vertices[-1][1]),
        sigma_x,
        sigma_y,
        edgecolor="red",
        facecolor="none",
    )
    ax_path.add_patch(ellipse)

    # Filter unique tags and plot
    unique_tags = {}
    for tag_id, tag in tags.items():
        if tag_id not in unique_tags:
            unique_tags[tag_id] = tag
        else:
            unique_tags[tag_id][0] = (unique_tags[tag_id][0] + tag[0]) / 2
            unique_tags[tag_id][1] = (unique_tags[tag_id][1] + tag[1]) / 2

    for tag_id, tag in unique_tags.items():
        ax_path.plot(tag[0], tag[1], "o", color=get_color(tag[3]))
        ax_path.text(tag[0], tag[1], str(tag[3]), color=get_color(tag[3]), fontsize=10)

    x_coords = [x for x, y in vertices] + [tag[0] for tag in tags.values()]
    y_coords = [y for x, y in vertices] + [tag[1] for tag in tags.values()]

    ax_path.set_xlim(min(x_coords) - 1, max(x_coords) + 1)
    ax_path.set_ylim(min(y_coords) - 1, max(y_coords) + 1)
    ax_path.set_aspect("equal")
    fig_path.canvas.draw()
    fig_path.canvas.flush_events()


def replay(directory: str):
    """Replay events from the directory."""
    with open(os.path.join(directory, "events.csv"), "r") as file:
        lines = file.readlines()

    mu_prev = np.array([0.0, 0.0, 0.0])
    Sigma_prev = np.eye(3) * 0.1
    acc_pos = [(0, 0)]
    detections = []
    tagss = {}

    prev_ltick = prev_rtick = prev_timestemp = False

    for line in lines[1000:]:
        timestemp, event, data = line.strip().split(",")
        timestemp = int(timestemp)

        if event == "left_wheel":
            curr_ltick = int(data)
            prev_ltick = prev_ltick if prev_ltick else curr_ltick
        elif event == "right_wheel":
            curr_rtick = int(data)
            prev_rtick = prev_rtick if prev_rtick else curr_rtick
        elif event == "image":
            img_path = os.path.join(directory, data)
            img = load_grayscale(img_path)
            detected_image = detect_tags(img)
            if not detected_image:
                print("No tags detected in image.")
                continue
            detections.append((timestemp, detected_image))
            bounding_boxes = [
                (x.center.tolist(), x.corners.tolist(), x.tag_id, x.pose_t)
                for x in detected_image
            ]
            visualize_bounding_boxes(img, bounding_boxes)

        if not prev_timestemp:
            prev_timestemp = timestemp

        delta_timestemp = timestemp - prev_timestemp
        if delta_timestemp > DELTA_TIME:
            prev_timestemp += DELTA_TIME

            delta_lphi = delta_phi(curr_ltick, prev_ltick, 135)
            delta_rphi = delta_phi(curr_rtick, prev_rtick, 135)
            prev_ltick, prev_rtick = curr_ltick, curr_rtick

            angular_disp, linear_disp = displacement(
                0.0318, 0.1, delta_lphi, delta_rphi
            )
            mu_prev, Sigma_prev, tags = estimate_pose(
                angular_disp,
                linear_disp,
                mu_prev,
                Sigma_prev,
                DELTA_TIME / 1e9,
                detections,
            )

            detections = []
            tagss = tagss | tags

            acc_pos.append((mu_prev[0], mu_prev[1]))
            plot_path(acc_pos, Sigma_prev[0, 0], Sigma_prev[1, 1], tagss)


def visualize_bounding_boxes(image: np.ndarray, bounding_boxes: List[Tuple]):
    """Visualize bounding boxes on an image."""
    ax_img.clear()
    ax_img.imshow(image, cmap="gray")
    for bbox in bounding_boxes:
        center, path, tag_id, _ = bbox
        ax_img.plot(center[0], center[1], "ro")
        polygon = Polygon(path, closed=True, edgecolor="blue", facecolor="none", lw=2)
        ax_img.add_patch(polygon)
        ax_img.text(
            center[0], center[1], str(tag_id), color=get_color(tag_id), fontsize=10
        )
    fig_img.canvas.draw()
    fig_img.canvas.flush_events()


# Detector initialization
detector = Detector(
    searchpath=["apriltags"],
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,  # adjust this to improve precision at more expensive cost
    quad_sigma=0.0,  # helps with noise, might reduce tag detection accuracy
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)

if __name__ == "__main__":
    args = parse_arguments()
    replay(args.dir)
