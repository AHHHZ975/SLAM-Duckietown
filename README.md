# The SLAM-Duckietown Research Project

This is the official repository for the SLAM-Duckietown - a project for the Autonomous Vehicles (Duckietown) Course in Fall 2024 at University of Montreal.

**Abstract**

This project implements an Extended Kalman Filter (EKF) SLAM algorithm for the Duckietown platform, a hands-on educational tool for learning autonomous robotics with small-scale robots (Duckiebots) in a miniature city. The goal is to estimate the Duckiebot's position and map static landmarks using data from wheel encoders and April tags. By integrating odometry with landmark detection, the system enhances localization accuracy and reduces drift over time.

The work focuses on offline SLAM tasks such as odometry estimation, April tag detection, and pose estimation, and extends to developing a ROS package for online experiments with the Duckiebot. Results demonstrate that the EKF-SLAM algorithm effectively improves localization and mapping, with performance directly influenced by the quality of April tag detection and the density of landmarks in the environment.

This project advances Duckietown's localization and mapping capabilities and aims to provide a foundation for building more robust and accurate SLAM systems. We hope that this code can help build a more robust and accurate SLAM system for the Duckietown platform.

**Contributors**:

- AmirHossein Zamani (amirhossein.zamani@mila.quebec)
- Léonard Oest O’Leary (leonard.oest.oleary@umontreal.ca, [@leo-ard](https://github.com/leo-ard))
- Kevin Lessard (kevin.lessard@umontreal.ca)

### Demonstration

https://github.com/user-attachments/assets/26e11008-5fd8-4916-b7ca-a924ae3cbd96





# Table of Contents
1. [Running the code](#running-the-code)
1. [Introduction](#introduction)
3. [Related Work](#related-work)
4. [Method](#method)
5. [Results](#results)
6. [Conclusion](#conclusion)
7. [References](#references)
8. [Annex](#annex)

# Running the code

There is two ways to run this project. Locally with prerecorded data or on a duckiebot. 
The implementation on the duckiebot has only been tested in the simulator, thus it may need some changes for it to work on a real duckiebot. 
We recommend that you run the code with the recorded data.

## Running it locally
Prerequisites:
- Python 3.12
- Linux or macOS
- [dts](https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_laptop/setup_dt_shell.html) command line package for Duckietown in python

## Offline EKF-SLAM Experiments

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

## Online EKF-SLAM Experiments in the Duckiematrix (no landmarks)

### 1. Clone the repository and cd
```bash
git clone https://github.com/AHHHZ975/SLAM-Duckietown.git
cd SLAM-Duckietown
```

### 2. Make sure you have a virtual duckiebot
You can create one with the command:

```
dts duckiebot virtual create [VBOT]
```

where `[VBOT]` can be anything you like (but remember it for later).

Then you can start your virtual robot with the command:

```
dts duckiebot virtual start [VBOT]
```

You should see it with a status `Booting` and finally `Ready` if you look at `dts fleet discover`: 

```
     | Hardware | Type    | Model     | Status | Hostname |
     | -------- | ------- | --------- | ------ | -------- | ------------ |
     | [VBOT]   | virtual | duckiebot | DB21J  | Ready    | [VBOT].local |
```

### 3. Make sure you update everything using these commands:
```bash
dts update
dts desktop udpate
dts duckiebot update [ROBOT_NAME]
```

### 4. Build the code on the robot:
```bash
dts code build -R [ROBOT_NAME]
```

### 5. Start the Duckiematrix simulator:

```
dts code start_matrix
```

### 6. Run the code on the robot:

```
dts code workbench -m -R [ROBOT_NAME]
```


In another terminal, you can launch the `noVNC` viewer for this exercise which can be useful to send commands to the robot and view the odometry that you calculating in the RViZ window. 

```
dts code vnc -R [ROBOT_NAME]
```

where `[ROBOT_NAME]` could be the real or the virtual robot (use whichever you ran the `dts code workbench` and `dts code build` command with).

*Running the code on the real robot is currently unsupported for 32GB Duckiebots. It might work if you have a 64GB SD card with a lot of space available. We plan to provide support for the real robot in the future as well as real world online experiments and compare them to our ofline counterparts.*


# Introduction


## Context
Duckietown is an open-source platform for accessible robotics and AI education, featuring a miniature urban environment where low-cost, autonomous Duckiebots navigate roads and intersections. Equipped with sensors like cameras, wheel encoders, and IMUs, Duckiebots primarily use a fisheye camera to detect April tags, enabling pose estimation and mapping of static landmarks.

Simultaneous Localization and Mapping (SLAM) is a fundamental task in autonomous robotics, involving the estimation of a robot's pose while simultaneously building a map of its environment. However, odometry-based motion estimation is susceptible to drift over time due to cumulative sensor errors. This project addresses these challenges by leveraging April tags as external landmarks to correct pose estimation errors, significantly reducing drift and improving the accuracy of localization and mapping.


## Objective
The primary objectives of this project are::

- To estimate the true pose (x, y, θ) of the Duckiebot over time.
- To determine the precise locations of static landmarks (like April tags) within the Duckietown environment.

To achieve these goals, we implemented an Extended Kalman Filter (EKF) SLAM algorithm that integrates data from wheel encoders and April tag detections. This approach improves pose estimation and mapping accuracy. The algorithm was evaluated in both offline and online settings, focusing on odometry estimation, April tag detection, and pose estimation.

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

The idea for this project originated from discovering an older online codebase on GitHub, **SLAMDuck** [4]. SLAMDuck is a detailed SLAM system specifically designed for the Duckietown platform, integrating odometry, April tag detection, and pose estimation. It provides a modular and extensible framework for implementing and evaluating various SLAM algorithms, serving as a valuable resource for the Duckietown community. However, the system encounters reproducibility challenges with the current version of the Duckietown platform. Additionally, the codebase lacks clear organization, and the reported results fall short of expectations. Despite these limitations, SLAMDuck offers potential for improvement and optimization.

Several research efforts have demonstrated the benefits of robust EKF-SLAM implementations and the use of landmarks to enhance performance. [5] introduces **TagSLAM**, a robust SLAM framework leveraging fiducial markers to improve localization accuracy. Unlike traditional EKF-based methods, TagSLAM utilizes a graph-based optimization approach, enabling it to incorporate observations from both past and future data, resulting in smoother and more accurate trajectory estimation. Similarly, [6] explores the implementation of an **EKF-based SLAM** system, providing detailed insights into both software and hardware phases. This study highlights the effectiveness of EKF in constructing accurate environmental maps by integrating sensor data and addresses common challenges in EKF-SLAM systems.

In addition, [7] provides a comprehensive overview of **probabilistic robotics**, covering foundational concepts such as Bayes filters, Kalman filters, and particle filters. This book served as a key reference for our implementation of the EKF-SLAM algorithm, particularly for incorporating landmark detection into the pipeline.

These works collectively highlight the advancements in EKF-SLAM systems utilizing fiducial markers like AprilTags. They offer valuable insights and techniques for developing robust and accurate localization and mapping solutions in autonomous robotics, guiding the design and optimization of our approach.

# Method

## Problem statement

The problem focuses on estimating two critical components for effective SLAM (Simultaneous Localization and Mapping):
- The **true pose** of the Duckiebot (x,y,θ) over time, derived from sensor inputs.
- The **positions** of static landmarks (x,y) within the Duckietown environment.

The Duckiebot collects a sequence of inputs, including odometry data from its wheel encoders and measurements from fiducial markers (April tags). Using these inputs, the challenge is to develop an algorithm that estimates both the Duckiebot's trajectory and the static landmark positions accurately. This estimation must address the inherent drift in motion estimation due to cumulative errors in odometry.

![image](https://github.com/user-attachments/assets/98828c24-0983-4d8f-ba1b-342c2c9280a6)

## SLAM = Pose Estimation (Odometry) + Landmarks
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

The pose estimation process combines odometry and April tag detections to predict and correct the Duckiebot's position and orientation. Encoder data provides an initial estimate of the Duckiebot's motion, while April tag measurements are used to refine these predictions and reduce drift, resulting in more accurate and reliable trajectory estimation.

### Interpolation Techniques

1) **Improved Interpolation**: The motion assumes a continuous arc, meaning linear and angular displacements are integrated over the movement, resulting in smoother and more accurate trajectory estimation.

2) **Simple Interpolation**: This assumes sequential application of linear and angular displacements, resulting in a more abrupt and less accurate trajectory, as seen in the red path in the previous section of the image.

![image](https://github.com/user-attachments/assets/267a8843-84e6-4040-98e1-c4dbe350c53b)

## Mathematical model:

The equations describe how the Duckiebot's position and orientation (x,y,θ) evolve over time based on its linear velocity (v) and angular velocity (ω). These formulas calculate the updated position after a small time step Δt, incorporating the Duckiebot's current orientation and motion. The updates rely on the kinematic model of a differential-drive robot, ensuring consistency with real-world motion dynamics.

![image](https://github.com/user-attachments/assets/c470354a-5367-4cf4-9d7e-3c300337c97b)


## April tags detection steps

April Tags play a key role in landmark-based SLAM by providing reliable fiducial markers for localization and mapping. The detection process involves the following steps:

1) **Image Preprocessing**: 
   - Convert the input image to grayscale.
   - Apply adaptive thresholding to enhance the contrast between the tag and the background.
   - Divide the image into tiles and compute local extrema to binarize the image, isolating regions of interest.
2) **Segmentation**: 
   - Detect edges by identifying pixels with significant intensity changes.
   - Use a union-find algorithm to cluster black and white pixels into distinct regions.
3) **Quadrilateral Detection**:
   - Fit quadrilateral shapes to the detected edges, as April tags are designed with a distinct square-like appearance.
4) **Decoding**:
   - Normalize the detected quadrilateral using homography to correct perspective distortions.
   - Extract the binary code embedded within the tag and decode it to retrieve the tag's unique ID.

![image](https://github.com/user-attachments/assets/01d8c9b2-e395-4689-b017-57c4b662e864)


## April tags pose estimation steps

April tags are crucial for determining the Duckiebot's pose relative to its environment. The process involves the following steps:

1) **Frame Definition**: 
   - Each April tag defines a local coordinate system with its center as the origin.
   - The four corners of the tag are assigned fixed 3D coordinates in this local frame, providing a consistent reference for pose estimation.

2) **Tag Detection**: 
   - Extract the tag's 2D pixel coordinates from the camera image using the detection method outlined earlier.
   - These coordinates represent the observed positions of the tag's corners in the image plane.
3) **PnP Solver**:
   - Use a Perspective-n-Point (PnP) solver to compute the transformation matrix that aligns the 3D coordinates of the tag's corners with their observed 2D positions in the camera's frame of reference.
   - This transformation matrix, consisting of rotation and translation components, provides precise localization of the tag relative to the camera.

By integrating this pose estimation data into the SLAM pipeline, the Duckiebot achieves consistent and reliable localization, enabling accurate positioning and mapping in its environment.

The diagrams below illustrate the coordinate transformations and the pose estimation workflow.

![image](https://github.com/user-attachments/assets/3ea9317b-b1f3-482e-87e5-0723022f8180)

![image](https://github.com/user-attachments/assets/1c2a0820-2858-4a9d-91a1-495c88f717c2)

## April tags Implementation Details

For this project, we utilized the **Duckietown AprilTag library** [8], which provides robust and efficient tools for detecting and decoding April tags in real-time. This library simplifies the integration of April tags into the SLAM pipeline and ensures reliable detection across various lighting and environmental conditions.

Details of the April tag detection process are illustrated above, and further information can be found in [2] (April Tags Documentation).

## Extended Kalman Filter

The Extended Kalman Filter (EKF) is a recursive algorithm that fuses sensor data to estimate the Duckiebot’s pose (x,y,θ) and the positions of static landmarks in the environment. It operates in two main steps — **prediction and correction** — running iteratively to balance the uncertainty of sensor data with the dynamics of the system. While conceptually straightforward, the EKF involves several nuances that can be counterintuitive:

### EKF Process:
1) **Prediction**: Use the motion model to predict the Duckiebot's next state based on odometry.
2) **Correction**: Incorporate April tag measurements to correct the prediction, reducing drift and improving accuracy.
3) **Covariance Update**: Adjust the uncertainty in the state estimate to reflect the confidence in the data.

By iteratively combining the predictive and corrective steps, the EKF ensures accurate localization and mapping, even in the presence of noisy or incomplete sensor inputs. The visual below illustrates the prediction-correction workflow in the EKF, showing how odometry and April tag data are fused for reliable pose estimation.

### Key Challenges
- **Linearization**: The EKF linearizes nonlinear motion and measurement models, but the error introduced depends on the operating region. For example, sharp turns or irregular tag placements can cause significant deviations from true dynamics.
- **Time-Varying Uncertainty**: As the Duckiebot moves, the uncertainty in its state grows during prediction steps and shrinks during correction. Understanding how these uncertainties propagate through the system requires a deep understanding of the covariance matrix dynamics.
- **Observability**: The EKF relies on sufficient measurements to reduce drift. If April tags are sparsely distributed or incorrectly detected, parts of the Duckiebot’s state may become unobservable, leading to unreliable estimates.

## Workflow

The visual below illustrates the EKF's prediction-correction workflow, showing how odometry data predicts the next state and April tag measurements refine it. By iteratively fusing these data sources, the EKF achieves accurate localization and mapping, even under noisy or incomplete sensor inputs.

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
https://github.com/user-attachments/assets/3858c440-4d17-4229-b8aa-2145974c0ea6

The first video showcases the combined process of April tag detection and odometry-based trajectory estimation. Using its fisheye camera, the Duckiebot identifies April tags in the environment through grayscale conversion and segmentation. The detected tags serve as fixed landmarks that correct the Duckiebot's trajectory, which is initially derived from odometry data collected via wheel encoders.

This integration reduces the cumulative drift often associated with odometry-only estimation. The video highlights how the SLAM system uses these external landmarks to refine pose accuracy, ensuring more reliable localization in the mapped environment.

## Offline April Tags' Position Estimation ([video link](https://www.youtube.com/watch?v=-cfKAiPUpaM))

https://github.com/user-attachments/assets/2ac35610-2712-431d-a631-2e3538d9de20

The second video demonstrates the estimation of April tags' positions relative to the Duckiebot. Detected 2D tag coordinates from the camera feed are matched with their known 3D coordinates using a Perspective-n-Point (PnP) solver. This process computes the transformation matrix, enabling precise estimation of the tags’ positions in the Duckiebot’s frame of reference.

The alignment between the observed and estimated tag positions validates the robustness of the pose estimation pipeline. The video further emphasizes the system's capability to maintain consistent localization and mapping, even as the Duckiebot moves through the environment.

## The comparison of Extended Kalman Filter SLAM (EKF-SLAM) results with the ground truth data

<table>
  <tr>
    <th>EKF-SLAM measurement model (without motion model)</th>
    <th>EKF-SLAM motion model (without measurement model)</th>
    <th>EKF-SLAM full version (motion+measurement model)</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/da79a185-7a2c-4f1f-a0b8-54e941080fed" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/bf5b45fa-734a-4904-a885-a5e5a4004a9e" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/7f4a74d4-a4f5-4d9d-a520-01ce36687e79" alt="Image 1" width="600"/></td>    
  </tr>
</table>


## The effect of different interpolation techniques on the pose trajectory

In both cases, the orange curve shows the prediction from EKF-SLAM motion model and the purple curve demonstrates the ground truty obtained from the Vicon system in the lab at UdeM. Here is the results with the ```DELTA_TIME=0.7``` seconds.

<table>
  <tr>
    <th>Pose estimation using a circular interpolation</th>
    <th>Pose estimation using a linear interpolation</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/5a9bfacd-be05-4214-b092-e59fb718a7f1" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/a97dae39-bb41-4edf-97f9-519d6f9586ea" alt="Image 1" width="600"/></td>    
  </tr>
</table>

The difference becomes more visible when increasing the delta time from 0.7 second to 2 seconds. Here is the results with the ```DELTA_TIME=2``` seconds.

<table>
  <tr>
    <th>Pose estimation using a circular interpolation</th>
    <th>Pose estimation using a linear interpolation</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/67c756cb-5d1c-4708-bb43-3f9b126b34c1" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/10f99306-697e-410f-9272-8346d5a3f8d8" alt="Image 1" width="600"/></td>    
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
    <td><img src="https://github.com/user-attachments/assets/514ad9c0-12e5-4571-aea1-af3a077becc4" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/0bb7b5e9-32c8-4fc0-bd5b-b61e769dcd3f" alt="Image 1" width="600"/></td>  
  </tr>
  
  The difference becomes more visible when increasing the delta time from 2 seconds to 5 seconds. Here is the results with the ```DELTA_TIME=5``` seconds.
  
  <tr>
    <th>0.7 sec</th>
    <th>1 sec</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/e5a66caf-a7e3-4e0c-a26f-a9fdd72b3955" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/4830ffa2-6b5e-4bcf-8412-337cecdfbd7c" alt="Image 1" width="600"/></td>
  </tr>
</table>

#### 2- Ablation on various amount of ```MEASUREMENT_MODEL_VARIANCE```
This study is done when the ```MOTION_MODEL_VARIANCE``` is kept fixed at the value of 0.1.
<table>
  <tr>
    <th>MEASUREMENT_MODEL_VARIANCE=0.05</th>
    <th>MEASUREMENT_MODEL_VARIANCE=0.2</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/1ed932e8-e9b7-41fa-b2e6-219aee980a93" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/81bf0dc4-726b-4a23-9045-7ae8c4de9199" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
    <th>MEASUREMENT_MODEL_VARIANCE=0.7</th>
    <th>MEASUREMENT_MODEL_VARIANCE=1.5</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/561e0041-c073-40c3-be7b-9156ce09b8a8" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/12a42350-a2ef-4a9a-a8b3-eecdff54f75c" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
</table>


#### 3- Ablation on various amount of ```MOTION_MODEL_VARIANCE```
This study is done when the ```MEASUREMENT_MODEL_VARIANCE``` is kept fixed at the value of 1.

<table>
  <tr>
    <th>MOTION_MODEL_VARIANCE=0.05</th>
    <th>MOTION_MODEL_VARIANCE=0.1</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/a15dc98a-2b6a-4360-896d-2a9ad3568767" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/05358078-3f73-4866-965b-04165f1e5fef" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
    <th>MOTION_MODEL_VARIANCE=0.5</th>
    <th>MOTION_MODEL_VARIANCE=1</th>
  </tr>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/f420ff0e-8353-4f33-8f5a-eb850c1a9106" alt="Image 1" width="600"/></td>
    <td><img src="https://github.com/user-attachments/assets/06284fed-93c7-43db-8096-fad729a6221e" alt="Image 1" width="600"/></td>
  </tr>
  <tr>
</table>
 
### Online Experiments on the virtual Duckiebot

The online experiments were conducted using the Duckiematrix simulator, which emulates the physical environment of Duckietown. The simulator provides a realistic setting for testing the EKF-SLAM algorithm in real-time. The online experiments focused on validating the SLAM system's accuracy and robustness in a simulated environment, providing insights into its real-world applicability. 

The figure below illustrates the setup for the online experiments, highlighting the key components of the simulation environment and the SLAM pipeline. However, we couldn't measure the accuracy of the online experiments due to the lack of ground truth data in the Duckiematrix simulator as well as the lack of landmarks. Also, the noise in the odometry data is almost negligeable in the simulation compared to the real world and is mainly affected by compute power. In the figure below, appart from the bad driving, the EKF-SLAM system is working as expected.

![Screenshot from 2024-12-23 21-46-49](https://github.com/user-attachments/assets/f8e1778a-4a5f-41ce-8929-d2ef8b85c0c5)
![Screenshot from 2024-12-23 22-29-00](https://github.com/user-attachments/assets/6303243f-9e24-4c3e-95ae-83d261959c69)

Attempts with the current version of Duckiebot failed at building the code properly, but we expect to bmake it work on the real Duckiebot in the future and test the system in the real world with the landmarks.

# Conclusion

This project successfully implemented an Extended Kalman Filter (EKF) SLAM algorithm for the Duckietown platform, demonstrating significant improvements in localization and mapping accuracy. By integrating odometry data with April tag detections, the algorithm effectively reduced drift and enhanced pose estimation through landmark-based corrections.

Comparative results showed measurable improvements over prior implementations [4], particularly when incorporating April tags as external landmarks. The exploration of different interpolation techniques and ablation studies provided valuable insights into the interplay between motion and measurement models, underscoring the critical role of parameter tuning in achieving robust SLAM performance.

While challenges were encountered in deploying the algorithm on the physical Duckiebot, the successful validation in simulation highlights the EKF-SLAM algorithm's real-world potential. Future work will focus on refining the system for seamless integration with hardware and expanding its applicability to more complex and dynamic environments.

# References

[1] [Lane-SLAM](https://github.com/mandanasmi/lane-slam)

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
Our first proposal involves understanding and re-implementing the [Lane-SLAM project](https://github.com/mandanasmi/lane-slam) [1] while adapting it to the current hardware and software versions of the Duckiebot and Duckietown. Similar to the original "Lane-SLAM" project, we aim to decouple the SLAM task—meaning we would compute localization and mapping steps independently. In this approach:

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
