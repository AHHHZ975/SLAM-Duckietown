
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.path import Path
import matplotlib.patches as patches

import argparse
import os

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



def replay(dir):
    file = open(os.path.join(dir, "events.csv"), "r")

    x_prev = 0
    y_prev = 0
    theta_prev = 0
    R = 0.0318  # meters, default value of wheel radius
    baseline = 0.1  # meters, default value of baseline

    prev_ltick = False
    prev_rtick = False

    curr_ltick = False
    curr_rtick = False

    resolution = 135
    prev_timestemp = False

    acc_pos = [(0, 0)]

    DELTA_TIME = 2_000_000_000 # 2 seconds

    lines = file.readlines()

    i = 0
    for line in lines[500:]:
        print("replaying image", (i:=i+1))
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
            bounding_boxes = list(map(lambda x : (x.center.tolist(), x.corners.tolist()), detected_image))
            visualize_bounding_boxes(img, bounding_boxes)

        timestemp = int(timestemp)
        if prev_timestemp == False:
            prev_timestemp = timestemp
        
        delta_timestemp = timestemp - prev_timestemp

        if delta_timestemp > DELTA_TIME:
            prev_timestemp = timestemp

            delta_lphi = delta_phi(curr_ltick, prev_ltick, resolution)
            delta_rphi = delta_phi(curr_rtick, prev_rtick, resolution)
            prev_ltick = curr_ltick
            prev_rtick = curr_rtick


            x_prev, y_prev, theta_prev = estimate_pose(
                R, 
                baseline, 
                x_prev, 
                y_prev, 
                theta_prev, 
                delta_lphi, 
                delta_rphi
            )
            acc_pos.append((x_prev, y_prev))
            plot_path(acc_pos)

        
    #plot(acc_pos)

def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
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


def estimate_pose(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    # These are random values, replace with your own

    linear_displacement_wheel_right = R * delta_phi_right
    linear_displacement_wheel_left = R * delta_phi_left
    linear_displacement_robot_origin = (linear_displacement_wheel_left + linear_displacement_wheel_right) / 2

    angular_displacement_robot_origin = (linear_displacement_wheel_right - linear_displacement_wheel_left) / (baseline)
    theta_curr = theta_prev + angular_displacement_robot_origin

    x_curr = x_prev + (linear_displacement_robot_origin*np.cos(theta_curr))
    y_curr = y_prev + (linear_displacement_robot_origin*np.sin(theta_curr))
    # ---
    return x_curr, y_curr, theta_curr

image_list = []
def plot_path(vertices):

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
    return detector.detect(img)
    
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
        center, path = bbox
        # Plot the center
        ax_img.plot(center[0], center[1], 'ro', label='Center' if 'Center' not in ax_path.get_legend_handles_labels()[1] else "")  # Red dot for the center

        # Create the polygon
        polygon = Polygon(path, closed=True, edgecolor='blue', facecolor='none', lw=2)
        ax_img.add_patch(polygon)
        
        # Optionally, label the center
        ax_img.text(center[0], center[1], 'Center', color='red', fontsize=9)

    ax_img.set_title("Image with Bounding Boxes")
    ax_img.axis("off")  # Turn off axis

    #fig.canvas.draw()
    fig_img.canvas.draw()
    fig_img.canvas.flush_events()
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



