# UoM AERO62520 Project (Group 4)

## Project Overview

The aim is to design, develop, and implement a mobile robot capable of autonomously re-trieving a brightly coloured wooden cube from its initial placement within a dynamic environment and depositing it into a predefined area.

## Hardware Components

- **Chassis (Leo Rover):** The mechanical base of the robot.
- **Lidar (RPLidar A2M12):** Lidar usedd for mapping and obstacle detection.
- **Depth Camera (Realsense D435i):** Provides depth perception for object detection and navigation.
- **Robotic Arm (Trossen-PincherX 150):** Responsible for picking up and manipulating objects.
- **Intel NUC:** The central processing unit for controlling and coordinating the robot's actions.

## Software Tools

**Primary Language** - Python \
**Framework** - ROS2 \
**Libraries:**
- ROS: leo_fw, realsense2_camera, rplidar_ros, curl, navigation2, slam_toolbox.
- Machine Learning - numpy, pandas, pytorch, scikitlearn.

**Simulation:**  RViz, Gazebo.

## Repository Structure

- **docs/ :** Project documentation.
  - *hardware:* Documentation related to the hardware design and configuration.
  - *software:* Documentation of intended software implementations to meet requirements. (work in progress)

- **src/ :** Source code for the robot's control and navigation.

- **ML_models/ :** Model training and data cleaning algorithms,  datasets and analytics.


## Usage

1. **Setup:**
   - Connect all hardware components according to the documentation in the `docs/README.md` folder.
   - Install any required dependencies as specified in the `docs/README.md`.

2. **Running the Robot:**
   - Instructions to start the robot will be updated in the `doc/README.md` .

3. **Emergency stop:**
   - Instructions to stop the robot with a priority control signal will be updated in `docs/README.md`

## Contributors

- Obianuju Ochuba
- Xingjian Zhang
- Feihan Hao
- Odysseas Bouziotis
  

