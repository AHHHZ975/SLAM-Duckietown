# SLAM-Duckietown

The official repository for the SLAM-Duckietown - a project for the Autonomous Vehicles (Duckietown) Course in Fall 2024 at University of Montreal.

**Abstract**

This project focuses on implementing an Extended Kalman Filter (EKF) SLAM algorithm for the Duckietown platform. Duckietown provides an educational and experimental environment for autonomous robotics, featuring small-scale robots (Duckiebots) navigating a miniature city. The objective is to estimate the Duckiebot's pose and the positions of static landmarks using its sensor inputs and motion data through wheel encoders. This work contributes to advancing localization and mapping capabilities in Duckietown. The project involves offline and online SLAM implementations, including odometry estimation, April tags detection, and pose estimation. The results demonstrate the effectiveness of the EKF-SLAM algorithm in improving the Duckiebot's localization and mapping performance.

**Contributors**:

- AmirHossein Zamani ()
- Léonard Oest O’Leary ()
- Kevin Lessard (kevin.lessard@umontreal.ca)


# Table of Contents
1. [Introduction](#introduction)
3. [Related Work](#related-work)
4. [Method](#method)
5. [Experiments and Reproducibility](#experiments-and-reproducibility)
6. [Results](#results)
7. [Reference](#reference)
8. [Annex](#annex)


# Introduction

## Context
The Duckietown project is an open-source initiative to democratize robotics and AI education. It offers a miniature city where small autonomous vehicles (Duckiebots) can navigate roads, intersections, and traffic signs. Duckiebots, low-cost and open-source robots, use cameras, wheel encoders, IMUs, and Time-of-Flight sensors for navigation. Equipped with fisheye cameras, they detect April tags, fiducial markers used for localization and mapping, to estimate their pose and the positions of static landmarks when combined with wheel encoder measurements.


## Objective
The goal of this project is to estimate:
- The true pose – θ (x, y, θ) – of the Duckiebot over time.
- The positions of static landmarks in the environment.

By achieving these objectives, the project aims to improve the Duckiebot’s localization and mapping capabilities within the Duckietown environment.

![image](https://github.com/user-attachments/assets/eed5114c-5de5-49d3-a039-1061505add9e)

## The overview of the project
![image](https://github.com/user-attachments/assets/433160b0-0fb7-46d8-bc35-9bc2dcd8b9d9)


## Motivation
Accurate localization and mapping are fundamental challenges in robotics, especially for low-cost, resource-constrained platforms like the Duckiebot. Implementing EKF-SLAM for Duckietown provides a practical framework for students and researchers to learn about simultaneous localization and mapping, while contributing to the broader goal of making robotics education accessible and impactful.


# Related Work


# Method



## Problem statement

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

## An overview of april tags detection steps [2]

- Image preprocessing: grayscale conversion and adaptive thresholding
- Segmentation: Union-find algorithm and clustering black and white pixels
- Quadrilateral detection: Flood-fill algorithm for connected components
- Decoding: Warping (homography), sampling for binary data, matching and centering

![image](https://github.com/user-attachments/assets/01d8c9b2-e395-4689-b017-57c4b662e864)


## April tags pose estimation

- Frame definition:
  - The center of the tag is chosen as the origin of the local coordinate system.
  - The four corners of the tag have fixed 3D coordinates in this local frame. 

- Extract the april tag position in the image using the tag Detection method (previous slide)

- PnP solver: Calculates the transformation matrix that maps the 3D tags coordinates to the camera's frame of reference.


![image](https://github.com/user-attachments/assets/3ea9317b-b1f3-482e-87e5-0723022f8180)

![image](`https://github.com/user-attachments/assets/1c2a0820-2858-4a9d-91a1-495c88f717c2`)



# Experiments and Reproducibility
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


# Results

## Offline April Tags Detection and Odometry Estimation ([video link](https://www.youtube.com/watch?v=KhHEgYVbUk4))
[![Watch the video](assets/Odometry_ApriltagsDetection.gif)](https://www.youtube.com/watch?v=KhHEgYVbUk4)

## Offline April Tags' Position Estimation ([video link](https://www.youtube.com/watch?v=KhHEgYVbUk4))
[![Watch the video](assets/Apriltag_pose_estimation.gif)](https://www.youtube.com/watch?v=KhHEgYVbUk4)


# Reference

[1] [Replica Dataset](https://github.com/facebookresearch/Replica-Dataset)

[2] [April Tags](https://april.eecs.umich.edu/software/apriltag)

[3] [Duckietown](https://www.duckietown.org/)


# Acknowledgement

We would like to thank the instructors of the Autonomous Vehicles (Duckietown) course at University of Montreal for their guidance and support throughout the project.

# Annex

## Annex 1: First Project Proposal Ideas

Here, we briefly outline the three potential directions we discussed for our project. These ideas generally fall into two categories:
### Visual SLAM


#### Project Proposal 1: Lane-SLAM Adaptation for Duckiebot
Our first proposal involves understanding and re-implementing the [Lane-SLAM project](https://github.com/mandanasmi/lane-slam) while adapting it to the current hardware and software versions of the Duckiebot and Duckietown. Similar to the original "Lane-SLAM" project, we aim to decouple the SLAM task—meaning we would compute localization and mapping steps independently. In this approach:

* **Mapping**: We start by detecting three types of lane markers (yellow segmented lines in the center, white sidelines, and red stop lines) from the camera image on the robot. We could use either a learning-based method or classical image-processing algorithms. We would then reconstruct these detected lines in 3D by computing the homography matrix between two consecutive image frames. Additionally, we can estimate camera motion through rotation and translation computations.

* **Localization**: Using data from the wheel encoders, we can track the relative robot position from its starting point within Duckietown/Duckiematrix.
Finally, by combining the robot’s position, 3D lane information, and camera motion data, we can visualize the environment as a map composed of lines and their respective colors. As a minor enhancement, we could incorporate IMU data (gyroscope and accelerometer) into odometry estimation by fusing the encoder and IMU data with Kalman or complementary filters to improve localization accuracy.


#### Project Proposal 2: Lightweight SLAM for Embedded Systems
Our second idea involves identifying and re-implementing an existing lightweight SLAM algorithm designed for embedded systems with limited computational resources. For reference, [this project by NVIDIA](https://nvidia-ai-iot.github.io/jetson_isaac_ros_visual_slam_tutorial/index.html) is an example of what we have in mind. As with the previous proposal, we would integrate IMU data to enhance localization accuracy.


### Non-Visual SLAM
#### Peroject Proposal 3:
To simplify the SLAM process, we could consider using non-visual data by relying solely on a ToF (Time-of-Flight) sensor to map the environment, eliminating the need for complex image processing. Here’s our plan for this approach:

* **Mapping**: By assuming the Duckietown/Duckiematrix environment has surrounding walls, we could gather useful ToF sensor data to detect these walls and create a map that includes walls and lanes. This would streamline the SLAM pipeline by reducing the complexity of camera-based lane detection.

* **Localization**: As with the other proposals, we could use both IMU and encoder data to determine the robot’s position.