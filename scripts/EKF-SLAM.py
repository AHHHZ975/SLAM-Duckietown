import numpy as np

# Initialize state and covariance
state = np.array([0, 0, 0])  # [x, y, theta]
covariance = np.eye(3) * 0.1  # Initial uncertainty

# Noise parameters
motion_noise = np.diag([0.1, 0.1, np.radians(5)])  # Motion noise [x, y, theta]
observation_noise = np.diag([0.5, 0.5])  # Observation noise [range, bearing]

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

# Main loop
timestamps = []  # List of timestamps
encoder_data = []  # List of [v, omega] pairs
observations = []  # List of [range, bearing] measurements
landmarks = []  # List of landmark positions [(x, y), ...]

for i in range(1, len(timestamps)):
    dt = timestamps[i] - timestamps[i - 1]
    control = encoder_data[i]

    # Prediction step
    state, covariance = predict(state, covariance, control, dt)

    # Update step (assuming one observation per timestamp)
    if observations[i] is not None:
        for obs, landmark in zip(observations[i], landmarks):
            state, covariance = update(state, covariance, obs, landmark)

    print(f"Time {timestamps[i]}: State: {state}, Covariance: {covariance}")
