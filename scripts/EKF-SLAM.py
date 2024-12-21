import numpy as np
import os
from typing import Tuple
import cv2
from dt_apriltags import Detector

# Initialize state and covariance
state = np.array([0, 0, 0])  # [x, y, theta]
covariance = np.eye(3) * 0.1  # Initial uncertainty

# Noise parameters
motion_noise = np.diag([0.1, 0.1, np.radians(5)])  # Motion noise [x, y, theta]
observation_noise = np.diag([0.5, 0.5])  # Observation noise [range, bearing]


detector = Detector(searchpath=['apriltags'],
                   families='tag36h11',
                   nthreads=1,
                   #max_hamming=max_hamming,
                   quad_decimate=1.0,
                   quad_sigma = 0.0,
                   refine_edges = 1,
                   decode_sharpening = 0.25,
                   debug=0)



# EKF Prediction Step
def predict(state, covariance, control, dt):
    v, omega = control  # Linear and angular velocities

    # Motion model
    theta = state[2]
    if abs(omega) > 1e-6:  # Avoid division by zero for straight motion
        dx = -v / omega * np.sin(theta) + v / omega * np.sin(theta + omega * dt)
        dy = v / omega * np.cos(theta) - v / omega * np.cos(theta + omega * dt)
        dtheta = omega * dt
    else:  # Straight motion
        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = 0

    # Update state
    state = state + np.array([dx, dy, dtheta])
    state[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))  # Normalize theta

    # Jacobian of motion model
    F = np.array([
        [1, 0, -v * dt * np.sin(theta)],
        [0, 1, v * dt * np.cos(theta)],
        [0, 0, 1]
    ])

    # Update covariance
    covariance = F @ covariance @ F.T + motion_noise
    return state, covariance

# EKF Update Step
def update(state, covariance, observation, landmark_pos):
    # Observation model
    dx = landmark_pos[0] - state[0]
    dy = landmark_pos[1] - state[1]
    q = dx**2 + dy**2
    z_hat = np.array([np.sqrt(q), np.arctan2(dy, dx) - state[2]])  # Expected observation

    # Jacobian of observation model
    H = np.array([
        [-dx / np.sqrt(q), -dy / np.sqrt(q), 0],
        [dy / q, -dx / q, -1]
    ])

    # Kalman Gain
    S = H @ covariance @ H.T + observation_noise
    K = covariance @ H.T @ np.linalg.inv(S)

    # Update state
    z = np.array(observation)  # Actual observation
    y = z - z_hat  # Innovation
    y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))  # Normalize angle
    state = state + K @ y

    # Update covariance
    covariance = (np.eye(len(covariance)) - K @ H) @ covariance
    return state, covariance

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

def load_grayscale(image_path):
    grayscale_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    return grayscale_image.astype(np.uint8)

def detect_tags(img):
    return detector.detect(img, estimate_tag_pose=True, camera_params=[340, 336, 328, 257], tag_size=0.05)

def read_csv_file(fileDir):
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

    # Read the csv file
    file = open(os.path.join(dir, fileDir), "r")
    lines = file.readlines()
    for line in lines[1000:]:
        timestamp, event, data = line.strip().split(",")        

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


# Main loop
all_timestamps = []  # List of timestamps
all_encoder_data = []  # List of [v, omega] pairs

# 1- Range:

#     This is the distance between the Duckiebot's camera and a landmark (AprilTag).
#     It is a scalar quantity and gives information about how far the landmark is from the Duckiebot.
#     For example, if the Duckiebot detects an AprilTag and calculates that it is 2 meters away, the range is 2 meters.

# 2- Bearing:

#     This is the angle (typically measured in radians or degrees) between a reference direction (usually the Duckiebot's forward-facing direction) and the line connecting the Duckiebot's camera to the landmark.
#     It provides directional information about where the landmark is located relative to the Duckiebot's orientation.
#     For example, if the AprilTag is detected slightly to the right of the Duckiebot's forward direction, the bearing might be +15°.
all_observations = []  # List of [range, bearing] measurements
all_landmarks = []  # List of landmark positions [(x, y), ...]

csv_file_dir = "events.csv"
read_csv_file(csv_file_dir)

for i in range(1, len(all_timestamps)):
    dt = all_timestamps[i] - all_timestamps[i - 1]
    control = all_encoder_data[i]

    # Prediction step
    state, covariance = predict(state, covariance, control, dt)

    # Update step (assuming one observation per timestamp)
    if all_observations[i] is not None:
        for obs, landmark in zip(all_observations[i], all_observations):
            state, covariance = update(state, covariance, obs, landmark)

    print(f"Time {timestamps[i]}: State: {state}, Covariance: {covariance}")
