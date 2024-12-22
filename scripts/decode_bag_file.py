#!/usr/bin/env python

# == HOW TO RUN THIS FILE ==
#
# This file needs to run in a ros environement. The easiest way to proceed (at least for the duckytown way) is to:
# (make sure you are in the root directory of SLAM-duckietown)
# $ dts start_gui_tools --mount $(pwd):/workdir
# $ cd /workdir
# $ python3 ./script/decode_bag_file -r <robot-name> -b ./bags/<bag-location> -o ./output/<output-location>
# $ echo You win!

import os
import rosbag
import cv2
import io
from duckietown_utils import rgb_from_ros

import argparse




def parse_arguments():
    parser = argparse.ArgumentParser(description="Process robot name, ROS bag location, and output directory.")
    
    # Add arguments
    parser.add_argument(
        "-r",
        "--robot_name",
        required=True,
        type=str,
        help="Name of the robot."
    )
    parser.add_argument(
        "-b",
        "--ros_bag_location",
        required=True,
        type=str,
        help="Path to the ROS bag file."
    )
    parser.add_argument(
        "-o",
        "--output_dir",
        required=True,
        type=str,
        help="Path to the output directory."
    )
    parser.add_argument(
        "-d",
        "--delay",
        required=False,
        type=int,
        default=0,
        help="Delay between the ground truth data and the ros bag data, in seconds and added to the ground truth data."
    )
    
    # Parse the arguments
    args = parser.parse_args()
    return args


def get_bag_events_and_write_images(bag, robot_name, output_dir):
    # load the bag
    
    
    # Ros topic to listen to
    ros_image_topic = f"/{robot_name}/camera_node/image/compressed"
    ros_left_wheel_topic = f"/{robot_name}/left_wheel_encoder_driver_node/tick"
    ros_right_wheel_topic = f"/{robot_name}/right_wheel_encoder_driver_node/tick"
    ros_camera_intrinsics = f"/{robot_name}/camera_node/camera_info"
    
    # File buffer
    events = []
    camera_intrinsics=None
    
    
    image_count = 0
    
    for topic,msg,t in bag.read_messages(topics = [ros_image_topic, ros_left_wheel_topic, ros_right_wheel_topic, ros_camera_intrinsics]):
        
        timestamp = t.to_time()
    
        if topic == ros_image_topic:
            # Decompress image from the bag file as frame{i}.png
            cv_img = rgb_from_ros(msg)
            image_name = "frame%06i.png" %image_count
            cv2.imwrite(os.path.join(output_dir, image_name), cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR))
            events.append((timestamp,"image",image_name))
    
            image_count += 1
        elif topic == ros_left_wheel_topic:
            # Left wheel encoder
            events.append((timestamp, "left_wheel", msg.data))
        elif topic == ros_right_wheel_topic:
            # Right wheel encoder
            events.append((timestamp, "right_wheel", msg.data))
        elif topic == ros_camera_intrinsics:
            # For recorded data, the camera intrinsics stay the same
            if camera_intrinsics is None:
                camera_intrinsics = msg

    first_timestamp = events[0][0]
    events.insert(0, (events[0][0]-1, "camera_intrinsis", [camera_intrinsics.D, camera_intrinsics.K, camera_intrinsics.R, camera_intrinsics.P]))

    return events, first_timestamp

def rotate_around(a, b, x, y, theta):
    from math import sin, cos
    return (a + (x-a) * cos(theta)-(y - b) * sin(theta), b + (x-a) * sin(theta) + (y - b) * cos(theta))

def get_ground_truth(first_timestamp, bag_prefix, delay):

    robot_position = f"{bag_prefix}.csv"
    landmarks_position = f"{bag_prefix}_trajectories.csv"

    # This file contains additionnal information such as the start time of the recordings
    system_information = f"{bag_prefix}.xcp"

    # Make sure all files exists
    if not all(map(os.path.exists, [robot_position, system_information, landmarks_position])):
        print("Didn't find ground truth data, skipping...")
        return [] # no additionnal events added

    import xml.etree.ElementTree as ET
    from datetime import datetime, timedelta

    # Find start time
    tree = ET.parse(system_information)
    root = tree.getroot()
    # correct for utc-5
    start_capture_time = datetime.fromisoformat(root.find("Camera/Capture").get("START_TIME")).timestamp() + timedelta(hours=5).total_seconds()
    end_capture_time = datetime.fromisoformat(root.find("Camera/Capture").get("END_TIME")).timestamp() + timedelta(hours=5).total_seconds()

    # ==========================================
    # ====  Decode all poses from the robot ==== 
    # ==========================================

    robot_pos_file = open(robot_position, "r")

    # Skip the 5 first line
    for _ in range(5):
        robot_pos_file.readline()
    
    r_positions = list(map(lambda x:x.split(","), robot_pos_file.read().strip().splitlines()))

    robot_pos_file.close()

    r_cor_positions = list(filter(lambda x : len(x) == 8, r_positions))
    framerate = round(len(r_positions) / (end_capture_time - start_capture_time))

    # Cast positions to int
    r_positions_int = list(map(lambda x: tuple(map(float, x)), r_cor_positions))

    # Use proper time
    r_position_time = list(map(lambda x : ((int(x[0]) / framerate) + start_capture_time + delay, *x[1:]), r_positions_int))

    # Find closest to position_time
    closest_time = sorted(r_position_time, key=lambda x: abs(x[0]-first_timestamp))[0]
    if abs(closest_time[0] - first_timestamp) > 2:
        print("Error, cannot proceed. Cannot find a matching time between the ground truth and the bag data. Use the delay to specify a delay between the two")
        print("Closest time found in ground truth: ", datetime.fromtimestamp(closest_time[0]).ctime())
        print("First event in the bag file:", datetime.fromtimestamp(first_timestamp).ctime())
        print("Diff: ", closest_time[0] - first_timestamp)
        exit(1)

    
    # Calculate robot path
    initial_pose = closest_time
    init_theta, init_x, init_y = initial_pose[4], initial_pose[5], initial_pose[6]
    
    new_poses = []
    vert = []

    from math import pi
    for pos in r_position_time:
        x, y = rotate_around(init_x, init_y, pos[5], pos[6], init_theta)
        norm_x = (x - init_x) / 1000 # normalize and convert to meters 
        norm_y = (y - init_y) / 1000

        new_poses.append((pos[0], "ground_truth", f"{norm_x}, {norm_y}"))
        vert.append((norm_x, norm_y))

    # ===============================
    # ==== Decode landmark poses ====
    # ===============================
    landmark_pos_file = open(landmarks_position, "r")

    for _ in range(5):
        landmark_pos_file.readline()
    
    # same as above
    l_positions = list(map(lambda x:x.split(","), landmark_pos_file.read().strip().splitlines()))
    landmark_pos_file.close()
    l_cor_positions = list(filter(lambda x : len(x) > 4, l_positions))
    l_positions_int = list(map(lambda x: tuple(map(lambda x: float(x) if x != '' else False, x)), l_cor_positions))
    l_position_time = list(map(lambda x : ((int(x[0]) / framerate) + start_capture_time + delay, *x[1:]), l_positions_int))

    landmark_count = len(l_position_time[0])//3
    landmark_positions = [[False, False, False, False] for _ in range(landmark_count)]
    for position in l_position_time:
        for count in range(landmark_count):
            x_count, y_count, z_count = 3*count+2, 3*count+3, 3*count+4
            pos_x, pos_y, pos_z = position[x_count], position[y_count], position[z_count]
            
            if pos_x != False:
                if landmark_positions[count][0] == False:
                    landmark_positions[count][0] = pos_x
                    landmark_positions[count][1] = pos_x
                else:
                    landmark_positions[count][0] = max(pos_x, landmark_positions[count][0])
                    landmark_positions[count][1] = min(pos_x, landmark_positions[count][1])

            if pos_y != False:
                if landmark_positions[count][2] == False:
                    landmark_positions[count][2] = pos_y
                    landmark_positions[count][3] = pos_y
                else:
                    landmark_positions[count][2] = max(pos_y, landmark_positions[count][2])
                    landmark_positions[count][3] = min(pos_y, landmark_positions[count][3])

    all_landmarks = []

    for landmark in landmark_positions:
        min_x, max_x, min_y, max_y = landmark
        if max_x-min_x > 20 or max_y - min_y > 20:
            print("Error, landmarks have moved too much")
            exit(1)

        pos_x, pos_y = (max_x+min_x) /2 ,(max_y+min_y) /2

        # try to find an existing landmark
        skip = False
        for land in all_landmarks:
            land_x, land_y = land
            if ((pos_x-land_x)**2 + (pos_y-land_y)**2) < 10**2: # bigger than 50mm or 5cm
                skip = True

        if skip:
            continue

        # normalize point
        x, y = rotate_around(init_x, init_y, pos_x, pos_y, init_theta)
        norm_x = (x - init_x) / 1000 # normalize and convert to meters 
        norm_y = (y - init_y) / 1000

        all_landmarks.append((norm_x, norm_y))

    
    plot_path(vert, all_landmarks)
    initial_timestamp=min(first_timestamp, new_poses[0][0])
    new_poses.insert(0, (first_timestamp, "landmarks", repr(all_landmarks)))
    
    return new_poses

# Check for ground truth data:


def plot_path(vertices, landmarks):

    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.path import Path
    import matplotlib.patches as patches

    #codes = [
    #    Path.MOVETO,  # Move to the starting point
    #    Path.LINETO,  # Draw a line
    #    Path.LINETO,  # Draw another line
    ##    Path.LINETO,  # Draw another line
    #]

    fig_path, ax_path = plt.subplots()
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
    #ellipse = patches.Ellipse((vertices[-1][0], vertices[-1][1]), sigma_x, sigma_y, edgecolor='red', facecolor='none')
    #ax_path.add_patch(ellipse)

    # Add points for the tags
    for tag in landmarks:
        ax_path.plot(tag[0], tag[1], 'ro', color="red")

    # Set limits and aspect ratio
    ax_path.set_xlim(min_x-1, max_x+1)
    ax_path.set_ylim(min_y-1, max_y+1)
    ax_path.set_aspect('equal')

    #fig_path.title("2D Canvas Path")

    # Display the plot
    #fig_path.canvas.draw()
    #fig_path.canvas.flush_events()
    plt.title("2D Canvas Path")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.grid(True)
    plt.plot()
    plt.savefig("foo.png")

def main():

    args = parse_arguments()
    print(f"Robot Name: {args.robot_name}")
    print(f"ROS Bag Location: {args.ros_bag_location}")
    print(f"Output Directory: {args.output_dir}")
    
    # create output_directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Open bag from location
    bag = rosbag.Bag(args.ros_bag_location)
    events, first_timestamp = get_bag_events_and_write_images(bag, args.robot_name, args.output_dir)
    bag.close()

    
    bag_basename = args.ros_bag_location[:-4] # remove ".bag"

    # Check for ground truth data
    ground_truth_events = get_ground_truth(first_timestamp, bag_basename, args.delay)

    # Open target file
    file = open(os.path.join(args.output_dir, "events.csv"), "w")

    all_events = sorted(ground_truth_events + events, key = lambda x:x[0]) # Sort on timestamp

    # Write all events:
    for event in all_events:
        if len(event) != 3:
            print("***error in event size")
            exit(1)
        data = event[2]
        if type(data) != str:
            data = repr(data).replace("\n","")
        file.write(str(event[0]) + "," + event[1] + ',' + data + "\n")

    #file.write(file_buffer.getvalue())
    
    # Close the events file
    file.close()

main()


