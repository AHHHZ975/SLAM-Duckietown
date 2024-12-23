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

![image](https://github.com/user-attachments/assets/c470354a-5367-4cf4-9d7e-3c300337c97b)


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

![image](https://github.com/user-attachments/assets/1c2a0820-2858-4a9d-91a1-495c88f717c2)


## Extended Kalman Filter
![Screenshot from 2024-12-20 18-09-33](https://github.com/user-attachments/assets/4dd33dd3-ee07-48f4-8103-ab388d64c816)


## Benchmarking using Vicon system


# Results

## Offline April Tags Detection and Odometry Estimation ([video link](https://www.youtube.com/watch?v=KhHEgYVbUk4))
[![Watch the video](readme_materials/Odometry_ApriltagsDetection.gif)](https://www.youtube.com/watch?v=KhHEgYVbUk4)

## Offline April Tags' Position Estimation ([video link](https://www.youtube.com/watch?v=KhHEgYVbUk4))
[![Watch the video](readme_materials/Apriltag_pose_estimation.gif)](https://www.youtube.com/watch?v=KhHEgYVbUk4)

## The comparison of Extended Kalman Filter SLAM (EKF-SLAM) results with the ground truth data

<table>
  <tr>
    <th>EKF-SLAM motion model (without measurement model)</th>
    <th>EKF-SLAM full version (motion+measurement model)</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/2e6e31eb-042a-4d8e-b112-f1225ff3c3f4" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/81e9e362-4eb5-461b-b8c6-cfb9e8371199" alt="Image 1" width="600"/></td>    
  </tr>
</table>


## The effect of different interpolation techniques on the pose trajectory

In both cases, the orange curve shows the prediction from EKF-SLAM motion model and the purple curve demonstrates the ground truty obtained from the Vicon system in the lab at UdeM. Here is the results with the ```DELTA_TIME=2``` seconds.

<table>
  <tr>
    <th>Pose estimation using a circular interpolation</th>
    <th>Pose estimation using a linear interpolation</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/d1f55d74-4531-4e06-acba-326ac7768c71" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/07f40984-d02b-44ff-bdcd-14aa7386656b" alt="Image 1" width="600"/></td>    
  </tr>
</table>

The difference becomes more visible when increasing the delta time from 2 seconds to 5 seconds. Here is the results with the ```DELTA_TIME=5``` seconds.



<table>
  <tr>
    <th>Pose estimation using a circular interpolation</th>
    <th>Pose estimation using a linear interpolation</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/6bec11d0-0c0c-4e30-b2ae-98ccde7de3a0" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/e0b748ed-436a-4c90-a15f-428f785ca7d2" alt="Image 1" width="600"/></td>    
  </tr>
</table>



## Ablation studies

#### 1- Ablation on various amount of ```DELTA_TIME```
<table>
  <tr>
    <th>0.05 sec</th>
    <th>0.2 sec</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/0a39dabd-cda4-4ba2-b257-170b3538406e" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/61e44ae4-aa71-49d8-8b40-087aa7520fa8" alt="Image 1" width="600"/></td>  
  </tr>
  <tr>
    <th>1 sec</th>
    <th>5 sec</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/c5e69082-9db9-45e5-9e68-ec592bf0e5b6" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/59f12939-e677-4bc7-9fd5-ed7a453fc768" alt="Image 1" width="600"/></td>
  </tr>
</table>

#### 2- Ablation on various amount of ```MEASUREMENT_MODEL_VARIANCE```
This study is done when the ```MOTION_MODEL_VARIANCE``` is kept fixed at the value of 0.1.
<table>
  <tr>
    <th>MEASUREMENT_MODEL_VARIANCE=0.01</th>
    <th>MEASUREMENT_MODEL_VARIANCE=0.2</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/a2112685-4cfa-4446-aeb0-9fe239be589e" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/6b778280-333c-49dd-bb5b-48aaa1cedff7" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
    <th>MEASUREMENT_MODEL_VARIANCE=1</th>
    <th>MEASUREMENT_MODEL_VARIANCE=10</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/86889e8d-2f44-49cb-8e78-dbeed6272f0c" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/b6397e00-d8bf-4132-9425-64e07df2f10f" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
</table>


#### 3- Ablation on various amount of ```MOTION_MODEL_VARIANCE```
This study is done when the ```MEASUREMENT_MODEL_VARIANCE``` is kept fixed at the value of 1.

<table>
  <tr>
    <th>MOTION_MODEL_VARIANCE=0.01</th>
    <th>MOTION_MODEL_VARIANCE=0.1</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/550a8972-7f85-4f69-9c2c-b318c0c81d8e" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/6f72dbc0-3d3a-487f-8c37-8ef1e2abc143" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
    <th>MOTION_MODEL_VARIANCE=0.3</th>
    <th>MOTION_MODEL_VARIANCE=1</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/63803a17-101f-4de4-8f46-bae28c11791d" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/10472a6d-11d3-41e0-8430-6f02d0cdb943" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
</table>






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

[2] [April Tags](https://april.eecs.umich.edu/software/apriltag)
