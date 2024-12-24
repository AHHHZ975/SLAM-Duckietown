# The SLAM-Duckietown Research Project
**Abstract**

This project implements an Extended Kalman Filter (EKF) SLAM algorithm for the Duckietown platform. 
Duckietown is a fun, hands-on way to learn about autonomous robotics using small-scale robots (Duckiebots) in a miniature city. 
Our goal is to estimate the Duckiebot’s position and the locations of static landmarks using data from wheel encoders and April tags.
By combining odometry with landmarks, we improve localization and reduce drift over time.
This work contributes to advancing localization and mapping capabilities in Duckietown.
We explored offline SLAM, focusing on key tasks like odometry estimation, April tag detection, and pose estimation.
We also created a ROS package for online experiments on the Duckiebot.
The results demonstrate the effectiveness of the EKF-SLAM algorithm in improving the Duckiebot's localization and mapping performance that is proportional to 
the quality of the April tag detection and the quantity of the landmarks in the environment. 
We hope that this code can help build a more robust and accurate SLAM system for the Duckietown platform.

**Contributors**:

- AmirHossein Zamani (amirhossein.zamani@mila.quebec)
- Léonard Oest O’Leary (leonard.oest.oleary@umontreal.ca, [@leo-ard](https://github.com/leo-ard))
- Kevin Lessard (kevin.lessard@umontreal.ca)

(This is the official repository for the SLAM-Duckietown - a project for the Autonomous Vehicles (Duckietown) Course in Fall 2024 at University of Montreal)

https://github.com/user-attachments/assets/f1a94873-0523-4f0e-9316-caa063d14dae


# Table of Contents
1. [Running the code](#running-code)
1. [Introduction](#introduction)
3. [Related Work](#related-work)
4. [Method](#method)
5. [Results](#results)
6. [Conclusion](#conclusion)
7. [Reference](#reference)
8. [Annex](#annex)

# Running code

There is two ways to run this project. Locally with prerecorded data or on a duckiebot. 
The implementation on the duckiebot has only been tested in the simulator, thus it may need some changes for it to work on a real duckiebot. 
We recommend that you run the code with the recorded data.

## Running it locally
Prerequisites:
- Python 3.12
- Linux or macOS
- [dts](https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_laptop/setup_dt_shell.html) command line

### 1. Clone the repository and cd
```bash
git clone https://github.com/AHHHZ975/SLAM-Duckietown.git
cd SLAM-Duckietown
```

### 2. Decompress the bag files into a readable format
```bash
dts start_gui_tools --mount $(pwd):/workdir
cd /workdir
./scripts/decompress_bag_file ./bags/<choose .bag file>
```

Alternatively, you can replace the last command with `python3 ./scripts/decode_bag_file.py`. See `python3 ./scripts/decode_bag_file.py --help` for options.



### 3. Install dependencies (Lower than python 3.13)

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

On macOS (or to have the latest version on linux):
```bash
chmod +x ./scripts/install_dt_apriltags
./scripts/install_dt_apriltags
pip install ./lib-dt-apriltags/dist/dt_apriltags-*.whl # as instructed by the script
```
If you have trouble, refer to the [lib-dt-apriltags repository](https://github.com/duckietown/lib-dt-apriltags)

### 4. Run the code.
```bash
python3 src/replay_no_ros.py -d ./output/<choose_dir>
```

### 5. (optional) Downloading more bag files
There is only one bag file available in the repository. More bag files are available for download [here](https://drive.google.com/drive/folders/1nS3F5duSDRVy3O2KG_plvYDwxeNxCxQr?usp=drive_link), or by running the following commands (in the root directory of the project) :

```bash
./scripts/download_bag_files
```

# Introduction


## Context
Duckietown is an open-source platform designed to make robotics and artificial intelligence education accessible to a broad audience. It simulates a miniature urban environment where small-scale autonomous vehicles, known as Duckiebots, navigate roads, intersections, and traffic infrastructure. Duckiebots are low-cost, open-source robots equipped with a range of sensors, including cameras, wheel encoders, IMUs, and Time-of-Flight sensors, enabling them to perceive and interact with their surroundings. The primary sensor used for localization and mapping is a fisheye camera, which detects April tags—fiducial markers placed in the environment. By combining April tag detections with wheel encoder measurements, Duckiebots can estimate their pose and map static landmarks.

Simultaneous Localization and Mapping (SLAM) is a critical task in autonomous robotics, aiming to estimate a robot's pose while simultaneously constructing a map of its environment. However, odometry-based motion estimation is prone to drift over time due to cumulative sensor errors. This project leverages April tags as external landmarks to correct pose estimation errors and mitigate drift, thereby improving the accuracy of localization and mapping.


## Objective
The primary objectives of this project are:

- To estimate the true pose (x, y, θ) of the Duckiebot over time.
- To determine the precise locations of static landmarks (like April tags) within the Duckietown environment.

To achieve these objectives, we propose an Extended Kalman Filter (EKF) SLAM algorithm that integrates data from wheel encoders and April tags. The EKF-SLAM algorithm combines odometry data with April tag detections to improve pose estimation and mapping accuracy. We aim to implement and evaluate the EKF-SLAM algorithm in both offline and online settings, focusing on key tasks like odometry estimation, April tag detection, and pose estimation.

## The overview of the project

This project focuses on the development and implementation of an Extended Kalman Filter (EKF) SLAM algorithm. The proposed approach integrates sensor data from wheel encoders and April tags to improve localization and mapping. Figure 1 illustrates the overall setup and methodology.

An overview of the project workflow, including the key components of the SLAM pipeline, is presented in the figure below. 

In stage 1, the process begins by separating the SLAM task into two main components: pose estimation (odometry) and landmark detection (April tags). The Duckiebot collects data from wheel encoders and camera images, which are used to estimate its motion and detect April tags in the environment. 

In stage 2, the EKF-SLAM algorithm fuses these data sources to estimate the Duckiebot's pose and the positions of static landmarks in the environment. The resulting trajectory and map provide valuable insights into the Duckiebot's localization and mapping capabilities.

![image](https://github.com/user-attachments/assets/433160b0-0fb7-46d8-bc35-9bc2dcd8b9d9)

## Primary Contributions

This project demonstrated that the EKF-SLAM algorithm significantly enhances the Duckiebot's localization and mapping capabilities by combining odometry data with April tag detections. The primary contributions are as follows:

1) **EKF-SLAM Implementation**: Developed and implemented an EKF-SLAM algorithm to improve pose estimation within a small-scale Duckietown environment, using offline bag files generated from Duckiebot movements controlled via joystick.
2) **Accuracy Evaluation**: Conducted a detailed comparison against ground truth data collected using the Vicon system to assess the accuracy and robustness of the EKF-SLAM algorithm.
3) **ROS Package Development**: Initiated the creation of a ROS package to enable real-time online experiments, paving the way for live SLAM performance evaluation on the Duckiebot.

# Related Work

The idea for this project originated from discovering an older online codebase on GitHub, SLAMDuck. [4] is a detailed SLAM system specifically designed for the Duckietown platform, integrating odometry, April tag detection, and pose estimation. It provides a modular and extensible framework for implementing and evaluating various SLAM algorithms, serving as a valuable resource for the Duckietown community. However, the system encounters reproducibility challenges with the current version of the Duckietown platform. Additionally, the codebase lacks clear organization, and the reported results fall short of expectations. Despite these limitations, SLAMDuck offers potential for improvement and optimization.

Several research efforts have demonstrated the benefits of robust EKF-SLAM implementations and the use of landmarks to enhance performance. [5] introduces TagSLAM, a robust SLAM framework leveraging fiducial markers to improve localization accuracy. Unlike traditional EKF-based methods, TagSLAM utilizes a graph-based optimization approach, enabling it to incorporate observations from both past and future data, resulting in smoother and more accurate trajectory estimation. Similarly, [6] explores the implementation of an EKF-based SLAM system, providing detailed insights into both software and hardware phases. This study highlights the effectiveness of EKF in constructing accurate environmental maps by integrating sensor data and addresses common challenges in EKF-SLAM systems.

In addition, [7] provides a comprehensive overview of probabilistic robotics, covering foundational concepts such as Bayes filters, Kalman filters, and particle filters. This book served as a key reference for our implementation of the EKF-SLAM algorithm, particularly for incorporating landmark detection into the pipeline.

These works collectively highlight the advancements in EKF-SLAM systems utilizing fiducial markers like AprilTags. They offer valuable insights and techniques for developing robust and accurate localization and mapping solutions in autonomous robotics, guiding the design and optimization of our approach.

# Method

## Problem statement

The problem focuses on estimating two critical components for effective SLAM (Simultaneous Localization and Mapping):
- The **true pose** of the Duckiebot (x,y,θ) over time, derived from sensor inputs.
- The **positions** of static landmarks (x,y) within the Duckietown environment.

The Duckiebot collects a sequence of inputs, including odometry data from its wheel encoders and measurements from fiducial markers (April tags). Using these inputs, the challenge is to develop an algorithm that estimates both the Duckiebot's trajectory and the static landmark positions accurately. This estimation must address the inherent drift in motion estimation due to cumulative errors in odometry.

![image](https://github.com/user-attachments/assets/98828c24-0983-4d8f-ba1b-342c2c9280a6)

## SLAM: Pose Estimation (Odometry) + Landmarks
SLAM involves two main components:
- **Pose Estimation (Odometry)**: This uses encoder data to estimate the Duckiebot's motion. However, odometry alone is prone to cumulative errors or drift over time.
- **Landmarks**: External references, such as April tags, provide additional observations to correct and refine odometry-based pose estimation.

The solution is to incorporate April tags as static landmarks in the environment to mitigate drift in motion estimation. By fusing landmark data with odometry, the overall accuracy of SLAM improves significantly.

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

The pose estimation process relies on a combination of odometry and April tag detections. The algorithm predicts the Duckiebot's position and orientation using encoder data. April tag measurements are then used to correct these predictions and reduce drift, resulting in a more reliable trajectory estimation.

![image](https://github.com/user-attachments/assets/267a8843-84e6-4040-98e1-c4dbe350c53b)

![image](https://github.com/user-attachments/assets/c470354a-5367-4cf4-9d7e-3c300337c97b)


## An overview of april tags detection steps

<details>
<summary>Overview of the april tags detection step</summary>
</details>
April Tags play a key role in landmark-based SLAM by providing reliable fiducial markers for localization and mapping. The detection process involves the following steps:
1) **Image Preprocessing**: The input image is converted to grayscale, and adaptive thresholding is applied to enhance contrast between the tag and the background. The image is divided into tiles, and local extrema are computed to binarize the image, effectively isolating regions of interest.
2) **Segmentation**: Edges are identified by detecting pixels with significant intensity changes. A union-find algorithm is then used to cluster black and white pixels into distinct regions.
3) **Quadrilateral Detection**: Quadrilateral shapes are fitted to the detected edges, as April Tags are designed to have a distinct square-like appearance.
4) **Decoding**: The detected quadrilateral is normalized using homography to correct perspective distortions. The binary code embedded within the tag is then extracted and decoded to retrieve the tag's unique ID.

![image](https://github.com/user-attachments/assets/01d8c9b2-e395-4689-b017-57c4b662e864)

To implement April tag detection in this project, we leveraged the Duckietown AprilTag library [8], which provides robust and efficient tools for identifying and decoding tags in real-time. This library simplifies the integration of April tags into the SLAM pipeline and ensures reliable detection in various lighting and environmental conditions. The details of the April tag detection process are illustrated below and more information can be found in [2] (April Tags Documentation).


## April tags pose estimation

April tags are crucial for determining the Duckiebot's pose relative to its environment. The process involves the following steps:

1) **Frame Definition**: Each April tag defines a local coordinate system, with the tag's center as the origin. The four corners of the tag are assigned fixed 3D coordinates in this local frame, providing a consistent reference for pose estimation.
2) **Tag Detection**: Using the detection method outlined earlier, the tag's 2D pixel coordinates are extracted from the camera image. This step provides the observed positions of the tag's corners in the image plane.
3) **PnP Solver**: A Perspective-n-Point (PnP) solver is used to compute the transformation matrix that aligns the 3D coordinates of the tag's corners with their observed 2D positions in the camera's frame of reference. This transformation matrix consists of rotation and translation components, enabling precise localization of the tag relative to the camera.

This pose estimation process provides the Duckiebot with accurate information about its position and orientation in the environment. By integrating this data with the SLAM pipeline, the Duckiebot can maintain consistent and reliable localization.

The steps are visualized in the diagrams below, illustrating the coordinate transformations and the pose estimation workflow.

![image](https://github.com/user-attachments/assets/3ea9317b-b1f3-482e-87e5-0723022f8180)

![image](https://github.com/user-attachments/assets/1c2a0820-2858-4a9d-91a1-495c88f717c2)


## Extended Kalman Filter

The Extended Kalman Filter (EKF) is a recursive algorithm that fuses sensor data to estimate the Duckiebot’s pose (x,y,θ) and the positions of static landmarks in the environment. It operates in two main steps — **prediction and correction** — which run iteratively. In the prediction step, the motion model uses odometry data from wheel encoders to estimate the Duckiebot’s next state, projecting its pose forward in time. This predicted state, however, is prone to errors due to sensor noise and drift. To address this, the correction step integrates measurements from April tags.

The final process involves:
1) **Prediction**: Use the motion model to predict the Duckiebot's next state based on odometry.
2) **Correction**: Incorporate April tag measurements to correct the prediction, reducing drift and improving accuracy.
3) **Covariance Update**: Adjust the uncertainty in the state estimate to reflect the confidence in the data.

By iteratively combining the predictive and corrective steps, the EKF ensures accurate localization and mapping, even in the presence of noisy or incomplete sensor inputs. The visual below illustrates the prediction-correction workflow in the EKF, showing how odometry and April tag data are fused for reliable pose estimation.


![Screenshot from 2024-12-20 18-09-33](https://github.com/user-attachments/assets/4dd33dd3-ee07-48f4-8103-ab388d64c816)


## Benchmarking using Vicon system

To benchmark our implementation, we compared the results of the SLAM prediction to ground truth data. The ground truth data was collected using a motion capture system (see [Vicon system](https://www.vicon.com/) for more information). This high-accuracy motion capture system can track objects at a millimeter precision, allowing us to have reliable ground truth data to compare our SLAM system to.

The procedure to collect this data was as following:
1. We prepared all landmarks and robot to have tracking balls on them
2. We started a recording of the robot odometry and camera using [rosbag](https://wiki.ros.org/rosbag)
3. At the same time, we start the recording of the motion capture system
4. We collected all files (bags, csv from tracking) in the repository and synchronize them (see [decode_bag_file.py](https://github.com/AHHHZ975/SLAM-Duckietown/blob/main/scripts/decode_bag_file.py) for implementation). Timestemp on both recordings was used to find the starting time of the robot and the ground truth data was reajusted to the duckiebot position and angle frame. Some recordings had a significant delay between the bag file and the ground truth CSV files, due to time desynchronization on the computer running the vicon system. In these cases, we resynchronized them by offsetting the timestemp by the number of seconds that differed between the computers.

# Results

## Offline April Tags Detection and Odometry Estimation ([video link](https://www.youtube.com/watch?v=KhHEgYVbUk4))
[![Watch the video](assets/Odometry_ApriltagsDetection.gif)](https://www.youtube.com/watch?v=KhHEgYVbUk4)

The first video showcases the combined process of April tag detection and odometry-based trajectory estimation. Using its fisheye camera, the Duckiebot identifies April tags in the environment through grayscale conversion and segmentation. The detected tags serve as fixed landmarks that correct the Duckiebot's trajectory, which is initially derived from odometry data collected via wheel encoders.

This integration reduces the cumulative drift often associated with odometry-only estimation. The video highlights how the SLAM system uses these external landmarks to refine pose accuracy, ensuring more reliable localization in the mapped environment.

## Offline April Tags' Position Estimation ([video link](https://www.youtube.com/watch?v=-cfKAiPUpaM))
[![Watch the video](assets/Apriltag_pose_estimation.gif)](https://www.youtube.com/watch?v=KhHEgYVbUk4)

The second video demonstrates the estimation of April tags' positions relative to the Duckiebot. Detected 2D tag coordinates from the camera feed are matched with their known 3D coordinates using a Perspective-n-Point (PnP) solver. This process computes the transformation matrix, enabling precise estimation of the tags’ positions in the Duckiebot’s frame of reference.

The alignment between the observed and estimated tag positions validates the robustness of the pose estimation pipeline. The video further emphasizes the system's capability to maintain consistent localization and mapping, even as the Duckiebot moves through the environment.

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
In both cases, the orange curve shows the prediction from EKF-SLAM motion model and the purple curve demonstrates the ground truty obtained from the Vicon system in the lab at UdeM. Here is the results with the ```DELTA_TIME=2``` seconds.

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
  
  The difference becomes more visible when increasing the delta time from 2 seconds to 5 seconds. Here is the results with the ```DELTA_TIME=5``` seconds.
  
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


# Reference

[1] [Replica Dataset](https://github.com/facebookresearch/Replica-Dataset)

[2] [April Tags](https://april.eecs.umich.edu/software/apriltag)

[3] [Duckietown](https://www.duckietown.org/)

[4] [SLAMDuck](https://github.com/asvath/SLAMDuck)

[5] [TagSLAM: Robust SLAM with Fiducial Markers](https://arxiv.org/pdf/1910.00679)

[6] [An Implementation of SLAM with Extended Kalman Filter](https://ieeexplore.ieee.org/document/7824105)

[7] [Probabilistic Robotics](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)

[8] [Duckietown AprilTag Library](https://github.com/duckietown/lib-dt-apriltags)


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
