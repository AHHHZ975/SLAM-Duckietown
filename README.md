# SLAM-Duckietown

The official repository for the SLAM-Duckietown- a project for the Autonomous Vehicles (Duckietown) Course in Fall 2024 at University of Montreal.

Contributors: AmirHossein Zamani, Léonard Oest O’Leary, and Kevin Lessard.

# Introduction

## Objective
![image](https://github.com/user-attachments/assets/eed5114c-5de5-49d3-a039-1061505add9e)

## The overview of the project
![image](https://github.com/user-attachments/assets/433160b0-0fb7-46d8-bc35-9bc2dcd8b9d9)


# Motivation


# Method

## Problem Statement

- Input: A sequence of the Duckiebot’s inputs and measurements
- Research Question: How to come up with an estimate of:
  - The true pose, (x, y, theta), of the Duckiebot at timesteps k1:k2
  - The estimate of the true positions (x,y) of the N static landmarks. 

![image](https://github.com/user-attachments/assets/98828c24-0983-4d8f-ba1b-342c2c9280a6)

## SLAM = pose estimation (odometry) + landmarks
- Issue: Motion estimation (odometry) drifts over time
- A potential solution: Use external landmarks in the environment to reduce/avoid drift

<table>
  <tr>
    <th>Incorporating landmarks into the odometry problem</th>
    <th>Odometry after incorporating landmarks</th>
    <th>Odometry before incorporating landmarks</th>    
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/5c98c612-9521-4a33-a9c2-65a8eb2d849c" alt="Image 1" width="400"/></td>
    <td><img src="https://github.com/user-attachments/assets/2e70adc6-db47-4831-a033-783c146c19c0" alt="Image 1" width="400"/></td>
    <td><img src="https://github.com/user-attachments/assets/640b9af3-138f-4374-94cc-66182df776e1" alt="Image 2" width="400"/></td>
  </tr>
</table>

## The overview of pose estimation
![image](https://github.com/user-attachments/assets/267a8843-84e6-4040-98e1-c4dbe350c53b)


# Results



## How to install

### 1. Clone the repository
```bash
git clone https://github.com/AHHHZ975/SLAM-Duckietown.git
```

### 2. Install dependencies

**(optional) Create a virtual environment**
```bash
python3 -m venv .venv
source .venv/bin/activate
```

**Install the required packages**
```bash
pip install -r requirements.txt
```

**Install dt-apriltags https://github.com/duckietown/lib-dt-apriltags**
On Ubuntu:
```bash
pip install dt-apriltags
```

On macOS (or to have the latest version):
```bash
chmod +x ./scripts/install_dt_apriltags
./scripts/install_dt_apriltags
pip install ./lib-dt-apriltags/dist/dt_apriltags-*.whl # as instructed by the script
```
If you have trouble, refer to the [lib-dt-apriltags repository](https://github.com/duckietown/lib-dt-apriltags)

### 3. Run the code. If you don't want to run through ros, you can run the following command:
```bash
python3 scripts/replay_no_ros.py -d ./output/<choose_dir>
```





Here, we briefly outline the three potential directions we discussed for our project. These ideas generally fall into two categories:
## Visual SLAM

### Project Proposal 1: Lane-SLAM Adaptation for Duckiebot
Our first proposal involves understanding and re-implementing the [Lane-SLAM project](https://github.com/mandanasmi/lane-slam) while adapting it to the current hardware and software versions of the Duckiebot and Duckietown. Similar to the original "Lane-SLAM" project, we aim to decouple the SLAM task—meaning we would compute localization and mapping steps independently. In this approach:

* **Mapping**: We start by detecting three types of lane markers (yellow segmented lines in the center, white sidelines, and red stop lines) from the camera image on the robot. We could use either a learning-based method or classical image-processing algorithms. We would then reconstruct these detected lines in 3D by computing the homography matrix between two consecutive image frames. Additionally, we can estimate camera motion through rotation and translation computations.

* **Localization**: Using data from the wheel encoders, we can track the relative robot position from its starting point within Duckietown/Duckiematrix.
Finally, by combining the robot’s position, 3D lane information, and camera motion data, we can visualize the environment as a map composed of lines and their respective colors. As a minor enhancement, we could incorporate IMU data (gyroscope and accelerometer) into odometry estimation by fusing the encoder and IMU data with Kalman or complementary filters to improve localization accuracy.

### Project Proposal 2: Lightweight SLAM for Embedded Systems
Our second idea involves identifying and re-implementing an existing lightweight SLAM algorithm designed for embedded systems with limited computational resources. For reference, [this project by NVIDIA](https://nvidia-ai-iot.github.io/jetson_isaac_ros_visual_slam_tutorial/index.html) is an example of what we have in mind. As with the previous proposal, we would integrate IMU data to enhance localization accuracy.

## Non-Visual SLAM
### Peroject Proposal 3:
To simplify the SLAM process, we could consider using non-visual data by relying solely on a ToF (Time-of-Flight) sensor to map the environment, eliminating the need for complex image processing. Here’s our plan for this approach:



* **Mapping**: By assuming the Duckietown/Duckiematrix environment has surrounding walls, we could gather useful ToF sensor data to detect these walls and create a map that includes walls and lanes. This would streamline the SLAM pipeline by reducing the complexity of camera-based lane detection.

* **Localization**: As with the other proposals, we could use both IMU and encoder data to determine the robot’s position.



# Reference
[1] [Replica Dataset](https://github.com/facebookresearch/Replica-Dataset)
[2] 
