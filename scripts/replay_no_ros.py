
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.path import Path
import matplotlib.patches as patches

import argparse
import os

from collections import defaultdict

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


UNIQUE_COLOR=['#e6194B', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#42d4f4', '#f032e6', '#bfef45', '#fabed4', '#469990', '#dcbeff', '#9A6324', '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', '#000075', '#a9a9a9',]
def get_color(id):
    return UNIQUE_COLOR[id%len(UNIQUE_COLOR)]

def replay(dir):
    file = open(os.path.join(dir, "events.csv"), "r")

    motion_model_mean = np.array([0.0, 0.0, 0.0]) # x, y, theta
    motion_model_covariance = np.eye(3) * 0.1
    R = 0.0318  # meters, default value of wheel radius
    baseline = 0.1  # meters, default value of baseline

    prev_ltick = False
    prev_rtick = False

    curr_ltick = False
    curr_rtick = False

    resolution = 135
    prev_timestemp = False

    acc_pos = [(0, 0)]

    DELTA_TIME = 500_000_000 # 2 seconds

    lines = file.readlines()
    #imgs = preload_images(dir, lines[500:])
    timestamp_detectedTags_pair_list = []
    tagss = {}

    i = 0
    for line in lines[1000:]:
        timestemp, event, data = line.strip().split(",")

        if event == "left_wheel":
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
            img = load_grayscale(img_path)
            detected_april_tags = detect_tags(img)
            timestamp_detectedTags_pair_list.append( (timestemp, detected_april_tags))
            #print("detect_tags", (datetime.now() - before).total_seconds())
            #pass
            bounding_boxes = list(map(lambda x : (x.center.tolist(), x.corners.tolist(), x.tag_id, x.pose_t), detected_april_tags))
            visualize_bounding_boxes(img, bounding_boxes)
            #print("\n".join(map(lambda x:f"{x.tag_id} : {x.pose_t}", detected_image)))
            #breakpoint()

        timestemp = int(timestemp)
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

            motion_model_mean, motion_model_covariance, tags = estimate_pose2(
                angular_displacement,
                linear_displacement,
                motion_model_mean,
                motion_model_covariance,
                DELTA_TIME / 1_000_000_000,
                timestamp_detectedTags_pair_list
            )
            timestamp_detectedTags_pair_list.clear()

            tagss = tagss | tags


            # Displaying the robot's trajectory
            acc_pos.append((motion_model_mean[0], motion_model_mean[1]))
            plot_path(acc_pos, motion_model_covariance[0,0], motion_model_covariance[1,1], tagss)
            plt.pause(0.05)
            #input("Press Enter to continue...")

        
    #plot(acc_pos)
    while True:
        plt.pause(0.05)

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

TAG_TO_INDEX = {}

IGNORE_TAGS = [74, 23, 26, 65] # These tags are duplicated in our experiments

def estimate_pose2(
    angular_displacement: float,
    linear_displacement: float,
    motion_model_mean: np.ndarray,
    motion_model_covariance: np.ndarray,
    delta_t: float,
    timestamp_detectedTags_pair_list : list
) -> Tuple[float, float, float]:
    

    detected_tags = defaultdict(list)
    for timestamp_detectedTags_pair in timestamp_detectedTags_pair_list:
        timestemp, detected_image = timestamp_detectedTags_pair

        for tag in detected_image:
            # We ignore ducplicate tags
            if tag.tag_id in IGNORE_TAGS:
                continue
            
            # "TAG_TO_INDEX" is a dictionary with the "key" equal to the tag id
            # and the "value" equalt to the index of that tag in the "detected_tags" list.
            if tag.tag_id not in TAG_TO_INDEX:
                TAG_TO_INDEX[tag.tag_id] = len(TAG_TO_INDEX)

            # Store the detected tags in a list of dictionaries with the following information:
            # 1- tag.pose_R: The rotation matrix in the transformation matrix between the camera and the tag
            # 2- tag.pose_t: The translation vector in the transformation matrix between the camera and the tag
            # 3- tag.pose_err: The error in the PnP solver when computing the transformation matrix
            detected_tags[TAG_TO_INDEX[tag.tag_id]].append([tag.pose_R, tag.pose_t, tag.pose_err])




    # Average each detected tag (maybe not a good way to do it)
    # What we are doing here is that, during a constant delta time,
    # for every tag we are looking for the occasions that the camera
    # has seen that specific tag. Then, we average the the translation
    # vector in the transformation matrix of these occasions and in this
    # way we are trying to have an estimation of the relative position of the
    # april tag with respect to the robot
    INV_TAG_TO_INDEX = {v: k for k, v in TAG_TO_INDEX.items()}
    tags = {}
    for tag_id, tag_data in detected_tags.items():
        #R = np.mean([tag[0] for tag in tag_data], axis=0)
        t = np.mean([tag[1] for tag in tag_data], axis=0)
        err = np.mean([tag[2] for tag in tag_data], axis=0)

        # Here, x and z are the relative postions of the april tag with respect to 
        # the specific robot
        x, z = t[0][0], t[2][0]

        # Here we are converting the relative position of the april tag 
        # to the global position in the world space.
        global_x = np.cos(motion_model_mean[2]) * z + np.sin(motion_model_mean[2]) * x
        global_y = np.sin(motion_model_mean[2]) * z - np.cos(motion_model_mean[2]) * x
        
        # Update the position of the april tag
        tags[tag_id] = [motion_model_mean[0]+global_x, motion_model_mean[1]+global_y, err, INV_TAG_TO_INDEX[tag_id]]



    # Resize the mu array if needed
    new_state_vector_size = 3 + 2 * len(TAG_TO_INDEX)
    old_state_vector_size = len(motion_model_mean)

    if len(motion_model_mean) < new_state_vector_size:
        motion_model_mean.resize(new_state_vector_size, refcheck=False)
        motion_model_covariance.resize((new_state_vector_size, new_state_vector_size), refcheck=False)
        
        # Initialize the new elements in the motion model's mean and covariance matrices
        for i in range(old_state_vector_size, new_state_vector_size):            
            motion_model_covariance[i, i] = 0.1
            tag_no = (i - 3) // 2
            motion_model_mean[3 + 2 * tag_no] = tags[tag_no][0]
            motion_model_mean[3 + 2 * tag_no + 1] = tags[tag_no][1]


    size = len(motion_model_mean)
    print("1 >>>>>>>>>>> mu", motion_model_mean)
    
    ###########################################################################################################
    ########################################### EKF Prediction Step ###########################################
    ###########################################################################################################    
    if abs(angular_displacement) <= 1e-6: # Linear movement        
        motion_model_mean[0] = motion_model_mean[0] + linear_displacement * np.cos(motion_model_mean[2])
        motion_model_mean[1] = motion_model_mean[1] + linear_displacement * np.sin(motion_model_mean[2])

    else: # Circular movement
        linear_to_angular_displacement_ratio = linear_displacement / angular_displacement
        # Move the robot
        motion_model_mean[0] = motion_model_mean[0] + linear_to_angular_displacement_ratio * (np.sin(motion_model_mean[2] + angular_displacement) - np.sin(motion_model_mean[2]))
        motion_model_mean[1] = motion_model_mean[1] + linear_to_angular_displacement_ratio * (np.cos(motion_model_mean[2]) - np.cos(motion_model_mean[2] + angular_displacement))
        motion_model_mean[2] = motion_model_mean[2] + angular_displacement

    # Jacobian of the motion model
    G = np.array([
        [1, 0, -linear_displacement * np.sin(motion_model_mean[2])],
        [0, 1, linear_displacement * np.cos(motion_model_mean[2])],
        [0, 0, 1]
    ])

    # Process noise covariance
    R_t = np.diag([0.1**2, 0.1**2, 0.1**2])  # Adjust noise values as needed

    # State mapping matrix
    F = np.zeros((3,size))
    F[0:3, 0:3] = np.eye(3)    

    # Covariance update
    G_F = F.T @ G @ F
    motion_model_covariance = G_F @ motion_model_covariance @ G_F.T + F.T @ R_t @ F
    
    ###########################################################################################################
    ########################################### EKF Update Step ###############################################
    ###########################################################################################################
    for tag_id, tag_pose in tags.items():

        # Line 12 of the EKF-SLAM algorithm      
        delta_x = tag_pose[0] - motion_model_mean[0]
        delta_y = tag_pose[1] - motion_model_mean[1]
        
        # Line 13 of the EKF-SLAM algorithm
        q = delta_x**2 + delta_y**2

        # Line 14 of the EKF-SLAM algorithm
        z = np.array([
            [np.sqrt(q)],
            [np.arctan2(delta_y, delta_x) - motion_model_mean[2]]
        ])

        # Line 15 of the EKF-SLAM algorithm
        F = np.zeros((5, size))
        F[0:3, 0:3] = np.eye(3)
        F[3:5, 3 + 2 * tag_id:3 + 2 * tag_id + 2] = np.eye(2)

        # Line 16 of the EKF-SLAM algorithm
        H = (np.array([
            [-np.sqrt(q) * delta_x, -np.sqrt(q) * delta_y, .0, np.sqrt(q) * delta_x, np.sqrt(q) * delta_y],
            [delta_y, -delta_x, -q, -delta_y, delta_x]
        ], dtype=float) / q) @ F

        # Line 17 of the EKF-SLAM algorithm
        # Notice that the Kalman gain is a matrix of size 3 by 3N + 3. This matrix is usually not sparse.
        K = motion_model_covariance @ H.T @ np.linalg.inv(H @ motion_model_covariance @ H.T +  0.1)
        
        # Line 18 of the EKF-SLAM algorithm       
        motion_model_mean = (motion_model_mean + (K@(np.array([[tag_pose[0]], [tag_pose[1]]]) - z)).T)[0]

        # Line 19 of the EKF-SLAM algorithm
        motion_model_covariance = (np.eye(size) - K @ H) @ motion_model_covariance
        
        #print(f"tag {tag[3]}: {tag[0], tag[1]}")

    return motion_model_mean, motion_model_covariance, tags

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
    #x_curr = x_prev + (linear_displacement_robot_origin*np.cos(theta_curr))
    #y_curr = y_prev + (linear_displacement_robot_origin*np.sin(theta_curr))
    ## ---
    #return x_curr, y_curr, theta_curr

image_list = []
def plot_path(vertices, sigma_x, sigma_y, tags):

    #codes = [
    #    Path.MOVETO,  # Move to the starting point
    #    Path.LINETO,  # Draw a line
    #    Path.LINETO,  # Draw another line
    ##    Path.LINETO,  # Draw another line
    #]

    ax_path.clear()
    # Create the Path object
    path = Path(vertices)

    max_x = max([x for x, y in vertices])
    max_y = max([y for x, y in vertices])
    min_x = min([x for x, y in vertices])
    min_y = min([y for x, y in vertices])

    # Create a PathPatch
    patch = patches.PathPatch(path, facecolor='orange', edgecolor='black', lw=2)

    # Set up the figure and axis
    ax_path.add_patch(patch)

    # Add variance ellipse
    ellipse = patches.Ellipse((vertices[-1][0], vertices[-1][1]), sigma_x, sigma_y, edgecolor='red', facecolor='none')
    ax_path.add_patch(ellipse)

    # Add points for the tags
    for tag_id, tag in tags.items():
        ax_path.plot(tag[0], tag[1], 'ro', color=get_color(tag[3]))
        ax_path.text(tag[0], tag[1], str(tag[3]), color=get_color(tag[3]), fontsize=25)

    # Set limits and aspect ratio
    ax_path.set_xlim(min_x-1, max_x+1)
    ax_path.set_ylim(min_y-1, max_y+1)
    ax_path.set_aspect('equal')

    #fig_path.title("2D Canvas Path")

    # Display the plot
    fig_path.canvas.draw()
    fig_path.canvas.flush_events()
    #plt.title("2D Canvas Path")
    #plt.xlabel("X-axis")
    #plt.ylabel("Y-axis")
    #plt.grid(True)
    #plt.plot()



from dt_apriltags import Detector
import cv2

def load_grayscale(image_path):
    grayscale_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    return grayscale_image.astype(np.uint8)


detector = Detector(searchpath=['apriltags'],
                   families='tag36h11',
                   nthreads=1,
                   #max_hamming=max_hamming,
                   quad_decimate=1.0,
                   quad_sigma = 0.0,
                   refine_edges = 1,
                   decode_sharpening = 0.25,
                   debug=0)


def detect_tags(img):
    return detector.detect(img, estimate_tag_pose=True, camera_params=[340, 336, 328, 257], tag_size=0.05)
    
print("hey")
plt.ion()
plt.show()
fig_img, ax_img = plt.subplots()
fig_path, ax_path = plt.subplots()


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
    #frame = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    #frame = frame.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    #image_list.append(frame)
    #plt.close(fig)  # Close the figure to free memory


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


args = parse_arguments()

replay(args.dir)

# def attempt(quad_sigma, decode_sharpening):
#     global image_list, detector
# 
#     image_list = []
#     detector = Detector(searchpath=['apriltags'],
#                        families='tag36h11',
#                        nthreads=1,
#                        #max_hamming=max_hamming,
#                        quad_decimate=1.0,
#                        quad_sigma = quad_sigma,
#                        refine_edges = 1,
#                        decode_sharpening = decode_sharpening,
#                        debug=0)
# 
#     replay(args.dir)
#     create_video_from_images(image_list, f"detected_april_tags_qs={quad_sigma}_ds={decode_sharpening}.mp4", 30)
# 
# #for quad_sigma in [0, 0.4, 0.8, 1.6, 3, 5]:
# #    for decode_sharpening in [0, 0.25, 0.5, 1, 5, 10]:
# #        #for max_hamming in [2, 4, 8]:
# #        print("running... qs=", quad_sigma, "ds=", decode_sharpening)
# #        attempt(quad_sigma, decode_sharpening)
# #


