
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


# def preload_images(dir, lines):
#     for line in lines:
#         timestemp, event, data = line.strip().split(",")
#         if event == "image":
#             img_path = os.path.join(dir, data)
#             img = load_grayscale(img_path)
#             detected_image = detect_tags(img)
#             #breakpoint()
#             #bounding_boxes = list(map(lambda x : (x.center.tolist(), x.corners.tolist()), detected_image))
#             #visualize_bounding_boxes(img, bounding_boxes)
#             #image_list.append(img)


UNIQUE_COLOR=['#e6194B', '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4', '#42d4f4', '#f032e6', '#bfef45', '#fabed4', '#469990', '#dcbeff', '#9A6324', '#fffac8', '#800000', '#aaffc3', '#808000', '#ffd8b1', '#000075', '#a9a9a9',]
def get_color(id):
    return UNIQUE_COLOR[id%len(UNIQUE_COLOR)]

def replay(dir):
    file = open(os.path.join(dir, "events.csv"), "r")

    mu_prev = np.array([.0, .0, .0]) # x, y, theta
    Sigma_prev = np.eye(3) * 0.1
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
    detections = []
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
            detected_image = detect_tags(img)
            detections.append( (timestemp, detected_image))
            #print("detect_tags", (datetime.now() - before).total_seconds())
            #pass
            bounding_boxes = list(map(lambda x : (x.center.tolist(), x.corners.tolist(), x.tag_id, x.pose_t), detected_image))
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
            print("delta_timestemp", delta_timestemp)
            prev_timestemp = prev_timestemp + DELTA_TIME

            delta_lphi = delta_phi(curr_ltick, prev_ltick, resolution)
            delta_rphi = delta_phi(curr_rtick, prev_rtick, resolution)
            prev_ltick = curr_ltick
            prev_rtick = curr_rtick

            angular_displacement, linear_displacement = displacement(
                R, 
                baseline, 
                delta_lphi, 
                delta_rphi
            )

            print("angular_displacement", angular_displacement)
            print("linear_displacement", linear_displacement)
            print("mu_prev", mu_prev)
            mu_prev, Sigma_prev, tags = estimate_pose2(
                angular_displacement,
                linear_displacement,
                mu_prev,
                Sigma_prev,
                DELTA_TIME / 1_000_000_000,
                detections
            )
            detections = []

            tagss = tagss | tags

            acc_pos.append((mu_prev[0], mu_prev[1]))
            plot_path(acc_pos, Sigma_prev[0,0], Sigma_prev[1,1], tagss)
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

def estimate_pose2(
    angular_displacement: float,
    linear_displacement: float,
    mu: np.ndarray,
    Sigma: np.ndarray,
    delta_t: float,
    detections : list
) -> Tuple[float, float, float]:

    print(len(detections))

    detected_tags = defaultdict(list)
    for detection in detections:
        timestemp, detected_image = detection

        for tag in detected_image:
            if tag.tag_id not in TAG_TO_INDEX:
                TAG_TO_INDEX[tag.tag_id] = len(TAG_TO_INDEX)
            detected_tags[TAG_TO_INDEX[tag.tag_id]].append([tag.pose_R, tag.pose_t, tag.pose_err])

    # Average each detected tag (maybe not a good way to do it)
    INV_TAG_TO_INDEX = {v: k for k, v in TAG_TO_INDEX.items()}
    tags = {}
    for tag_id, tag_data in detected_tags.items():
        #R = np.mean([tag[0] for tag in tag_data], axis=0)
        t = np.mean([tag[1] for tag in tag_data], axis=0)
        err = np.mean([tag[2] for tag in tag_data], axis=0)
        x, z = t[0], t[2]

        # Here z looks forward and x looks to the right
        #rel_x = np.cos(mu[2]) * x + np.sin(mu[2]) * z
        #rel_y = np.sin(mu[2]) * x - np.cos(mu[2]) * z

        print(mu[2])

        rel_x = np.cos(mu[2]) * z + np.sin(mu[2]) * x
        rel_y = np.sin(mu[2]) * z - np.cos(mu[2]) * x
        
        tags[tag_id] = [mu[0]+rel_x, mu[1]+rel_y, err, INV_TAG_TO_INDEX[tag_id]]

        #print("tag_id", tag_id)
        #print("R", R)
        #print("t", t)
        #print("err", err)

    

    #print("detected_tags", detected_tags.keys())


    if angular_displacement == 0:
        inter_vw = linear_displacement
        mu[0] = mu[0] + inter_vw * np.cos(mu[2])
        mu[1] = mu[1] + inter_vw * np.sin(mu[2])

        # Update sigma
        # Jacobian of the motion model
        G = np.array([  
           [1, 0, -inter_vw * np.sin(mu[2])],
           [0, 1, inter_vw * np.cos(mu[2])],
           [0, 0, 1]
        ])
    else:
        inter_vw = linear_displacement / angular_displacement
    
        # Move the robot
        mu[0] = mu[0] + inter_vw * (np.sin(mu[2] + angular_displacement) - np.sin(mu[2]))
        mu[1] = mu[1] + inter_vw * (np.cos(mu[2]) - np.cos(mu[2] + angular_displacement))
        mu[2] = mu[2] + angular_displacement

        # Update sigma
        # Jacobian of the motion model
        G = np.array([  
           [1, 0, -inter_vw * np.cos(mu[2]) + inter_vw * np.cos(mu[2] + angular_displacement)],
           [0, 1, -inter_vw * np.sin(mu[2]) + inter_vw * np.sin(mu[2] + angular_displacement)],
           [0, 0, 1]
        ])

    Sigma = G @ Sigma @ G.T + np.eye(3) * 0.05 * delta_t
    return mu, Sigma, tags

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



