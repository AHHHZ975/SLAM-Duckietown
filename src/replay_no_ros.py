# This file is a utility to replay recorded events from the bag file without the need of ROS dependencies.

from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.path import Path
import matplotlib.patches as patches
from dt_apriltags import Detector
import cv2
import argparse
import os
from collections import defaultdict

MOTION_MODEL_VARIANCE = 0.1
MEASUREMENT_MODEL_VARIANCE = 0.7
DELTA_TIME = 0.7 # second
ENABLE_MEASUREMENT_MODEL = True
ENABLE_CIRCULAR_INTERPOLATION = True

# Be warned, this option gives unprivileged information to the ekf algorithmq
# (a.k.a, the ekf algorithm is aware of the ground truth position of the tags)
ENABLE_GOD_EKF = False
# The 'secret key' is the correspondance between the tag numbers and the positions on the map

GOD_SECRET_KEY = [80, 44, 41, 26, 110, 106, 101, 31, 65, 23, 232, 54]

DISABLE_MOTION_MODEL = False # This option disables the motion model
ENABLE_FAST_MODE = False # (may be less precise)
ENABLE_CAMERA_VISUALIZATION = True # This makes the runtime so much faster, if you don't care about the 
                                   # camera images and april tags bounding boxes visualization.



image_list = []
IGNORE_TAGS = []
# IGNORE_TAGS = [74, 23, 26, 65] # These tags are duplicated in our experiments




# Command line utility
def parse_arguments():
    parser = argparse.ArgumentParser(description="Process robot name, ROS bag location, and output directory.")
    
    # Add arguments
    parser.add_argument(
        "-d",
        "--dir",
        required=True,
        type=str,
        help="Decoded directory"
    )
    # Parse the arguments
    args = parser.parse_args()
    return args


def get_color(id):
    # Colors to represent the april tags in our results.
    unique_colors=['#e6194B', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#42d4f4', '#f032e6', '#bfef45', '#fabed4', '#469990', '#dcbeff', '#9A6324', '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', '#000075', '#a9a9a9',]
    return unique_colors[id%len(unique_colors)]


# Main replay function
def replay(dir):
    file = open(os.path.join(dir, "events.csv"), "r")

    motion_model_mean = np.array([0.0, 0.0, 0.0]) # x, y, theta
    motion_model_covariance = np.eye(3) * MOTION_MODEL_VARIANCE
    R = 0.0318  # meters, default value of wheel radius
    baseline = 0.1  # meters, default value of baseline

    prev_ltick = False
    prev_rtick = False

    curr_ltick = False
    curr_rtick = False

    resolution = 135
    prev_timestemp = False

    acc_pos = [(0, 0)]

    TAG_INDEX = {}

    lines = file.readlines()
    timestamp_detectedTags_pair_list = []
    measured_tags = {}
    ground_truth_positions = []
    list_of_landmarks = False

    camera_params = [340, 336, 328, 257]

    i = 0
    for line in lines:
        timestemp, event, data, *rest = line.strip().split(",")

        if event == "ground_truth":
            x, y = float(data), float(rest[0])
            ground_truth_positions.append((x, y))

        elif event == "left_wheel":
            data = int(data)
            if prev_ltick == False:
                prev_ltick = data
                curr_ltick = data
            else:
                curr_ltick = data
            

        elif event == "right_wheel":
            data = int(data)
            if prev_rtick == False:
                prev_rtick = data
                curr_rtick = data
            else:
                curr_rtick = data

        elif event == "image":
            img_path = os.path.join(dir, data)
            if ENABLE_FAST_MODE:
                image_list.append((timestemp, img_path))
            else:
                img = load_grayscale(img_path)
                detected_april_tags = detect_tags(img, camera_params)
                timestamp_detectedTags_pair_list.append((timestemp, detected_april_tags))
                bounding_boxes = list(map(lambda x : (x.center.tolist(), x.corners.tolist(), x.tag_id, x.pose_t), detected_april_tags))
                
                if ENABLE_CAMERA_VISUALIZATION:
                    visualize_bounding_boxes(img, bounding_boxes)

        elif event == "landmarks":
            first_coma = line.find(",")
            second_coma = line.find(",", first_coma+1)

            # Do not look at the following line. It's not good practice, and mostly a quick and dirty way to do it.
            list_of_landmarks = eval(line[second_coma+1:])

            if ENABLE_GOD_EKF:

                new_size = len(list_of_landmarks) * 2 + 3

                # Sets the motion model mean to the ground truth values
                new_motion_model_mean = np.zeros(new_size)
                new_motion_model_mean[0:2] = motion_model_mean[0:2]
                for i, landmark in enumerate(list_of_landmarks):
                    new_motion_model_mean[3+i*2] = landmark[0]
                    new_motion_model_mean[3+i*2+1] = landmark[1]
                motion_model_mean = new_motion_model_mean

                # Set the variance of all tags to zero
                new_motion_model_covariance = np.zeros((new_size, new_size))
                new_motion_model_covariance[0:3, 0:3] = motion_model_covariance
                motion_model_covariance = new_motion_model_covariance

                TAG_INDEX = {GOD_SECRET_KEY[i]:i for i in range((new_size-3)//2)}


        elif event == "camera_intrinsis":
            first_coma = line.find(",")
            second_coma = line.find(",", first_coma+1)

            # Do not look at the following line. It's not good practice, and mostly a quick and dirty way to do it.
            camera_intrinsics = eval(line[second_coma+1:])

            # Here, camera_intrisics is a list containing the K matrix (calibration matrix) in it's second field (camera_intrinsics[1])
            # The matrix is as follows
            #
            # [ f_x 0   t_x ]
            # [ 0   f_y t_y ]
            # [ 0   0   1   ]
            # 
            # In our case, the K matrix is represented as a vector in ONE dimention. We can thus see that :
            #  f_x = index 0
            #  f_y = index 4
            #  t_x = index 2
            #  t_y = index 5
            # 
            #  camera_params is then used in the dt-apriltags library which need the data in this format [f_x, f_y, t_x, t_y]. See dt-apriltag repo for more information.
            K = camera_intrinsics[1]
            camera_params = [K[0], K[4],K[2],K[5]]
            #breakpoint()

        else:
            print("Unknown event")
            print(event)
            exit(1)

        timestemp = float(timestemp)
        if prev_timestemp == False:
            prev_timestemp = timestemp
        
        delta_timestemp = timestemp - prev_timestemp

        if delta_timestemp > DELTA_TIME:
            i += 1
            print(">>>> i", i)            
            prev_timestemp = prev_timestemp + DELTA_TIME

            delta_lphi = delta_phi(curr_ltick, prev_ltick, resolution)
            delta_rphi = delta_phi(curr_rtick, prev_rtick, resolution)
            prev_ltick = curr_ltick
            prev_rtick = curr_rtick

            # Odometry estimation 
            angular_displacement, linear_displacement = displacement(
                R, 
                baseline, 
                delta_lphi, 
                delta_rphi
            )

        
            # Only process the last 5 images in "fast mode"
            if ENABLE_FAST_MODE:
                timestamp_detectedTags_pair_list.clear()
                last_images = image_list[-5:]
                for img_pair in last_images:
                    img_path = img_pair[1]
                    timestamp = img_pair[0]
                    img = load_grayscale(img_path)
                    detected_april_tags = detect_tags(img, camera_params)
                    timestamp_detectedTags_pair_list.append((timestemp, detected_april_tags))
                    bounding_boxes = list(map(lambda x : (x.center.tolist(), x.corners.tolist(), x.tag_id, x.pose_t), detected_april_tags))
                    if ENABLE_CAMERA_VISUALIZATION:                    
                        visualize_bounding_boxes(img, bounding_boxes)

            motion_model_mean, motion_model_covariance, measured_tags = EKF_pose_estimation(
                angular_displacement,
                linear_displacement,
                motion_model_mean,
                motion_model_covariance,
                DELTA_TIME,
                timestamp_detectedTags_pair_list,
                TAG_INDEX
            )
            timestamp_detectedTags_pair_list.clear()

            # Uncomment the following line if you want the measured tags to stay
            #measured_tags = measured_tags | tags


            # Displaying the robot's trajectory
            acc_pos.append((motion_model_mean[0], motion_model_mean[1]))

            plot_path(acc_pos, ground_truth_positions, list_of_landmarks, motion_model_mean, motion_model_covariance, measured_tags, TAG_INDEX)
            plt.pause(0.00001)            

def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> float:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """

    # TODO: these are random values, you have to implement your own solution in here
    delta_ticks = ticks - prev_ticks    
    alpha = 2*np.pi/resolution
    delta_phi = delta_ticks * alpha
    # ---
    return delta_phi


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

def displacement(
    R: float,
    baseline: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float]:

    linear_displacement_wheel_right = R * delta_phi_right
    linear_displacement_wheel_left = R * delta_phi_left
    linear_displacement = (linear_displacement_wheel_left + linear_displacement_wheel_right) / 2

    angular_displacement = (linear_displacement_wheel_right - linear_displacement_wheel_left) / (baseline)

    return angular_displacement, linear_displacement

def plot_path(vertices, ground_truth_vertices, ground_truth_landmarks, model_mean, model_covariance, measured_tags, TAG_INDEX):
    from matplotlib import rcParams


    # Set global font size
    rcParams.update({'font.size': 18})  # Increase the global font size to 18

    ax_path.clear() # Reset canvas

    # Create the Path object
    path = Path(vertices)
    ground_truth_path = Path(ground_truth_vertices)

    # Determine the size of the canvas
    max_x = max([x for x, y in vertices] + [0.5])
    max_y = max([y for x, y in vertices] + [1])
    min_x = min([x for x, y in vertices] + [-0.5])
    min_y = min([y for x, y in vertices] + [0])

    # Create a PathPatch
    patch = patches.PathPatch(path, facecolor="none", edgecolor='orange', lw=5, label="Estimated Robot Pose")
    gt_patch = patches.PathPatch(ground_truth_path, facecolor="none", edgecolor='navy', lw=2, label="Ground Truth Robot Pose")

    # Add patches to the axis
    ax_path.add_patch(patch)
    ax_path.add_patch(gt_patch)

    # Plot landmarks if present
    if ground_truth_landmarks:
        for land in ground_truth_landmarks:
            ax_path.plot(land[0], land[1], 'ro', color="navy", label="Ground Truth Landmark Pose")


    # Add variance ellipse around the robot, and a dot for the robot
    sigma_x = model_covariance[0][0]
    sigma_y = model_covariance[1][1]
    ellipse = patches.Ellipse((vertices[-1][0], vertices[-1][1]), sigma_x, sigma_y, edgecolor='gold', facecolor='none', label="Variance Ellipse")
    ax_path.add_patch(ellipse)
    ax_path.plot(vertices[-1][0], vertices[-1][1], 'rs', color='darkred')


    for i in range(3, len(model_mean), 2):
        tag_x = model_mean[i]
        tag_y = model_mean[i+1]
        tag_vx = model_covariance[i][i]
        tag_vy = model_covariance[i+1][i+1]
        
        INV_TAG_TO_INDEX = {v: k for k, v in TAG_INDEX.items()}
        tag_index = INV_TAG_TO_INDEX[(i-3)//2]
        
        ax_path.plot(tag_x, tag_y, 'ro', color=get_color(tag_index))
        ellipse = patches.Ellipse((tag_x, tag_y), tag_vx, tag_vy, edgecolor='gold', facecolor='none', label="Variance Ellipse")
        ax_path.add_patch(ellipse)
        ax_path.text(tag_x, tag_y, str(tag_index), color=get_color(tag_index), fontsize=10)  # Increase text font size for tags

    # Plot measured tags and their text
    for tag_id, tag in measured_tags.items():
        ax_path.plot(tag[0], tag[1], 'rs', color='green', label="Measurement from AprilTags")
        ax_path.plot([vertices[-1][0], tag[0]], [vertices[-1][1], tag[1]], 'k-')
        #ax_path.text(tag[0], tag[1], str(tag[3]), color='green', fontsize=10)  # Increase text font size for tags


    # Set limits and aspect ratio
    ax_path.set_xlim(min_x - 1, max_x + 1)
    ax_path.set_ylim(min_y - 1, max_y + 1)
    ax_path.set_aspect('equal')

    # Add axis labels
    ax_path.set_xlabel("X position (m)", fontsize=20)
    ax_path.set_ylabel("Y position (m)", fontsize=20)

    # Add a title to the figure
    fig_path.suptitle("Robot and Landmark Pose Estimation Visualization Using EKF-SLAM Algorithm", fontsize=24)

    # Add the legend with increased font size
    handles, labels = ax_path.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))  # Remove duplicates
    ax_path.legend(by_label.values(), by_label.keys(), loc='upper right', fontsize=16)

    # Update the figure
    fig_path.canvas.draw()
    fig_path.canvas.flush_events()






def load_grayscale(image_path):
    grayscale_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    return grayscale_image.astype(np.uint8)



# Camera intrinsic parameters on the Apriltag's website: 336.7755634193813, 333.3575643300718, 336.02729840829176, 212.77376312080065
# Our robot's camera intrinsic parameters: 340, 336, 328, 257
def detect_tags(img, camera_params):
    return detector.detect(img, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.065)


def visualize_bounding_boxes(image, bounding_boxes):
    """
    Visualizes an image with bounding boxes overlaid.

    :param image: NumPy array of the image.
    :param bounding_boxes: List of bounding boxes, where each box is a tuple:
                           (center, path).
                           - center: (cx, cy) coordinates of the center.
                           - path: List of (x, y) coordinates defining the polygon.
    """

    ax_img.clear()
    ax_img.imshow(image, cmap='gray')  # Display the image in grayscale if single-channel

    for bbox in bounding_boxes:
        center, path, tag_id, pose_t = bbox
        # Plot the center
        ax_img.plot(center[0], center[1], 'ro', label='Center' if 'Center' not in ax_path.get_legend_handles_labels()[1] else "")  # Red dot for the center

        # Create the polygon
        polygon = Polygon(path, closed=True, edgecolor='blue', facecolor='none', lw=2)
        ax_img.add_patch(polygon)
        
        # Optionally, label the center
        ax_img.text(center[0], 
                    center[1], 
                    str(tag_id), #+ ":" + ",".join(map(lambda x: (f'{x[0]:.2}'), pose_t.tolist())), 
                    color=get_color(tag_id), 
                    fontsize=25)

    ax_img.set_title("Image with Bounding Boxes")
    ax_img.axis("off")  # Turn off axis

    #fig.canvas.draw()
    fig_img.canvas.draw()
    fig_img.canvas.flush_events()
    plt.show()



def create_video_from_images(image_list, video_path, fps=10):
    """
    Create a video from a list of images.
    
    :param image_list: List of NumPy arrays (images).
    :param video_path: Path to save the output video.
    :param fps: Frames per second for the video.
    """
    height, width, _ = image_list[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
    video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    for img in image_list:
        video_writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))  # Convert RGB to BGR for OpenCV

    video_writer.release()
    print(f"Video saved to {video_path}")


if __name__ == "__main__":
    detector = Detector(searchpath=['apriltags'],
                    families='tag36h11',
                    nthreads=1,
                    #max_hamming=max_hamming,
                    quad_decimate=1.0,
                    quad_sigma = 0.0,
                    refine_edges = 1,
                    decode_sharpening = 0.25,
                    debug=0)


    plt.ion()            
    plt.show()
    if ENABLE_CAMERA_VISUALIZATION:        
        fig_img, ax_img = plt.subplots()
    fig_path, ax_path = plt.subplots()
    # fig_error, ax_error = plt.subplots(figsize=(10, 6))


    args = parse_arguments()

    replay(args.dir)
            
    while True:
        plt.pause(0.05)
