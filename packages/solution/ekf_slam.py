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

MOTION_MODEL_VARIANCE = 0.1
MEASUREMENT_MODEL_VARIANCE = 0.7
DELTA_TIME = 2 # second
ENABLE_MEASUREMENT_MODEL = True
ENABLE_CIRCULAR_INTERPOLATION = True
# Be warned, this option gives unprivileged information to the ekf algorithmq
# (a.k.a, the ekf algorithm is aware of the ground truth position of the tags)
ENABLE_GOD_EKF = False
# The 'secret key' is the correspondance between the tag numbers and the positions on the map
GOD_SECRET_KEY = [80, 44, 41, 26, 110, 106, 101, 31, 65, 23, 232, 54]
DISABLE_MOTION_MODEL = False # This option disables the motion model
ENABLE_FAST_MODE = False # (may be less precise)
ENABLE_CAMERA_VISUALIZATION = False # This makes the runtime so much faster, if you don't care about the 
                                    # camera images and april tags bounding

image_list = []
IGNORE_TAGS = []
# IGNORE_TAGS = [74, 23, 26, 65] # These tags are duplicated in our experiments


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

def EKF_pose_estimation(
    angular_displacement: float,
    linear_displacement: float,
    motion_model_mean: np.ndarray,
    motion_model_covariance: np.ndarray,
    delta_t: float,
    timestamp_detectedTags_pair_list : list,
    TAG_INDEX
    ) -> Tuple[float, float, float]:


    detected_tags = defaultdict(list)
    for timestamp_detectedTags_pair in timestamp_detectedTags_pair_list:
        timestemp, detected_image = timestamp_detectedTags_pair

        for tag in detected_image:
            # We ignore ducplicate tags
            if tag.tag_id in IGNORE_TAGS:
                continue

            if (tag.pose_t[2][0] ** 2 + tag.pose_t[0][0]**2) > (1.5) ** 2:
                continue
            
            # "TAG_TO_INDEX" is a dictionary with the "key" equal to the tag id
            # and the "value" equal to the index of that tag in the "detected_tags" list.
            if tag.tag_id not in TAG_INDEX:
                TAG_INDEX[tag.tag_id] = len(TAG_INDEX)

            # Store the detected tags in a list of dictionaries with the following information:
            # 1- tag.pose_R: The rotation matrix in the transformation matrix between the camera and the tag
            # 2- tag.pose_t: The translation vector in the transformation matrix between the camera and the tag
            # 3- tag.pose_err: The error in the PnP solver when computing the transformation matrix
            detected_tags[TAG_INDEX[tag.tag_id]].append([tag.pose_R, tag.pose_t, tag.pose_err])


    # Average each detected tag (maybe not a good way to do it)
    # What we are doing here is that, during a constant delta time,
    # for every tag we are looking for the occasions that the camera
    # has seen that specific tag. Then, we average the the translation
    # vector in the transformation matrix of these occasions and in this
    # way we are trying to have an estimation of the relative position of the
    # april tag with respect to the robot
    INV_TAG_TO_INDEX = {v: k for k, v in TAG_INDEX.items()}
    tags_positions = {}
    for tag_id, tag_data in detected_tags.items():
        # phi = np.mean([tag[0] for tag in tag_data], axis=0)
        t = np.mean([tag[1] for tag in tag_data], axis=0)
        
        err = np.mean([tag[2] for tag in tag_data], axis=0)

        # Here, x and z are the relative postions of the april tag with respect to 
        # the specific robot
        x_tag_relative_to_robot, y_tag_relative_to_robot = t[2][0], -t[0][0]
        # x_tag_relative_to_robot, y_tag_relative_to_robot = t[0][0], t[2][0] # what it was before        

        # Here we are converting the relative position of the april tag 
        # to the global position in the world space. The origin of the
        # world coordinate is set at the start position of the Duckiebot.
 
        # Update the position of the april tag
        range_tag = np.sqrt(x_tag_relative_to_robot**2 + y_tag_relative_to_robot**2)
        bearing_tag = np.arctan2(y_tag_relative_to_robot, x_tag_relative_to_robot) # The bearing is the angle between the tag's position and the forward direction of the camera in the 2D plane
        tags_positions[tag_id] = [motion_model_mean[0] + (range_tag * np.cos(bearing_tag + motion_model_mean[2])),
                        motion_model_mean[1] + (range_tag * np.sin(bearing_tag + motion_model_mean[2])),
                        err,
                        INV_TAG_TO_INDEX[tag_id],
                        range_tag,
                        bearing_tag
                    ]



    # Resize the mu array if needed
    new_state_vector_size = 3 + 2 * len(TAG_INDEX)
    old_state_vector_size = len(motion_model_mean)

    if len(motion_model_mean) < new_state_vector_size:
        new_motion_model_mean = np.zeros(new_state_vector_size)
        new_motion_model_mean[0:old_state_vector_size] = motion_model_mean
        motion_model_mean = new_motion_model_mean

        new_motion_model_covariance = np.zeros((new_state_vector_size, new_state_vector_size))
        new_motion_model_covariance[0:old_state_vector_size, 0:old_state_vector_size] = motion_model_covariance
        motion_model_covariance = new_motion_model_covariance
        
        # Initialize the mean covariancethe new elements in the motion model's mean and covariance matrices
        for i in range(old_state_vector_size, new_state_vector_size, 2):
            motion_model_covariance[i, i] = 10000     # The initial cov for the landmarks is inf because we have no idea where they are
            motion_model_covariance[i+1, i+1] = 10000 # The initial cov for the landmarks is inf because we have no idea where they are
            tag_no = (i - 3) // 2
            motion_model_mean[3 + 2 * tag_no] = tags_positions[tag_no][0] # x position of april tag
            motion_model_mean[3 + 2 * tag_no + 1] = tags_positions[tag_no][1] # y position of april tags


    size = len(motion_model_mean)
    
    ###########################################################################################################
    ########################################### EKF Prediction Step ###########################################
    ###########################################################################################################        
    num_landmarks = (size - 3) // 2 # Number of landmarks in the state
    Fx = np.hstack((np.eye(3), np.zeros((3, 2 * num_landmarks)))) # Define Fx (state-to-map matrix)
    previous_robot_orientation = motion_model_mean[2]
    
    if DISABLE_MOTION_MODEL:
        G = np.eye(3)
    else:
        if ENABLE_CIRCULAR_INTERPOLATION: # When we're using the circular interpolation
            if abs(angular_displacement) <= 1e-2: # Linear movement 

                motion_model_mean = motion_model_mean + Fx.T @ np.array([
                    linear_displacement * np.cos(previous_robot_orientation),
                    linear_displacement * np.sin(previous_robot_orientation),
                    0
                ])     

                # Jacobian
                G = np.array([
                [1, 0, -linear_displacement * np.sin(previous_robot_orientation)],
                [0, 1, linear_displacement * np.cos(previous_robot_orientation)],
                [0, 0, 1]
                ])
            else: # Circular movement
                linear_to_angular_displacement_ratio = linear_displacement / angular_displacement
                motion_model_mean = motion_model_mean + (Fx.T @ np.array([
                    -linear_to_angular_displacement_ratio * np.sin(previous_robot_orientation) + linear_to_angular_displacement_ratio * np.sin(previous_robot_orientation + angular_displacement),
                    linear_to_angular_displacement_ratio * np.cos(previous_robot_orientation) - linear_to_angular_displacement_ratio * np.cos(previous_robot_orientation + angular_displacement),
                    angular_displacement
                ]))        
                motion_model_mean[2] = (motion_model_mean[2] + np.pi) % (2 * np.pi) - np.pi # Keep theta in [-pi, pi]

                # Jacobian
                G = np.array([  
                [1, 0, -linear_to_angular_displacement_ratio * np.cos(previous_robot_orientation) + linear_to_angular_displacement_ratio * np.cos(previous_robot_orientation + angular_displacement)],
                [0, 1, -linear_to_angular_displacement_ratio * np.sin(previous_robot_orientation) + linear_to_angular_displacement_ratio * np.sin(previous_robot_orientation + angular_displacement)],
                [0, 0, 1]
                ])
        else: # When we're using the linear interpolation
            motion_model_mean = motion_model_mean + Fx.T @ np.array([
                linear_displacement * np.cos(previous_robot_orientation),
                linear_displacement * np.sin(previous_robot_orientation),
                angular_displacement
            ])

            # Jacobian
            G = np.array([
            [1, 0, -linear_displacement * np.sin(previous_robot_orientation)],
            [0, 1, linear_displacement * np.cos(previous_robot_orientation)],
            [0, 0, 1]
            ])


    # Noise covariance matrix for the motion model
    R = np.diag([MOTION_MODEL_VARIANCE**2, MOTION_MODEL_VARIANCE**2, (MOTION_MODEL_VARIANCE/2)**2])

    # State mapping matrix
    F = np.zeros((3,size))
    F[0:3, 0:3] = np.eye(3)

    # Covariance update
    G_F = np.eye(size)  
    G_F[0:3, 0:3] = G 
    motion_model_covariance = G_F @ motion_model_covariance @ G_F.T + F.T @ R @ F

    ###########################################################################################################
    ########################################### EKF Update Step ###############################################
    ###########################################################################################################    
    if ENABLE_MEASUREMENT_MODEL:
        for tag_id, tag_pose in tags_positions.items():
            # Line 6 of the EKF-SLAM algorithm
            Q = np.diag([MEASUREMENT_MODEL_VARIANCE**2, MEASUREMENT_MODEL_VARIANCE**2]) # Noise covariance matrix for the measurement model

            tag_index = 3 + 2 * tag_id

            # Line 12 of the EKF-SLAM algorithm 
            delta = motion_model_mean[tag_index:tag_index+2] - motion_model_mean[0:2]     
            
            # Line 13 of the EKF-SLAM algorithm
            q = delta.T @ delta

            # Line 14 of the EKF-SLAM algorithm
            range_tag, bearing_tag = tag_pose[4:6]
            z_actual = np.array([range_tag, bearing_tag]) # Actual observation from sensors
            z_estimation = np.array([ # The estimation of observation
                np.sqrt(q),
                np.arctan2(delta[1], delta[0]) - motion_model_mean[2]]
            )
            z_diff = z_actual - z_estimation 
            # print(z_actual[0]-z_estimation[0])
            # print(z_actual[1]-z_estimation[1])
            z_diff[1] = (z_diff[1] + np.pi) % (2 * np.pi) - np.pi # Normalize the angle in the observation difference to fall within the range [−π,π]

            # Line 15 of the EKF-SLAM algorithm
            Fx_j = np.zeros((5, size))
            Fx_j[0:3, 0:3] = np.eye(3)
            Fx_j[3:5, tag_index:tag_index + 2] = np.eye(2)

            # Line 16 of the EKF-SLAM algorithm
            H = (np.array([
                [-np.sqrt(q) * delta[0], -np.sqrt(q) * delta[1], .0, np.sqrt(q) * delta[0], np.sqrt(q) * delta[1]],
                [delta[1], -delta[0], -q, -delta[1], delta[0]]
            ], dtype=float) / q) @ Fx_j

            # Line 17 of the EKF-SLAM algorithm
            # Notice that the Kalman gain is a matrix of size 3 by 3N + 3. This matrix is usually not sparse.        
            K = motion_model_covariance @ H.T @ np.linalg.inv(H @ motion_model_covariance @ H.T + Q)

            # Line 18 of the EKF-SLAM algorithm             
            motion_model_mean += K @ z_diff
            #breakpoint()

            # Line 19 of the EKF-SLAM algorithm
            motion_model_covariance = (np.eye(size) - K @ H) @ motion_model_covariance

    return motion_model_mean, motion_model_covariance, tags_positions


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
