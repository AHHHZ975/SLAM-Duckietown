# SLAM-Duckietown

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
