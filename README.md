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

- **docs/ :** Detailed project documentation.
  - *hardware:* Documentation related to the hardware design and configuration.
  - *software:* Documentation of intended software implementations to meet requirements.

- **src/ :** Contains the source code for the robot's control and navigation.
  - *README.md:* Instructions for running and modifying the code.


## Usage

1. **Setup:**
   - Connect all hardware components according to the documentation in the `docs/hardware/README.md` folder.
   - Install any required dependencies as specified in the `code/README.md`.

2. **Running the Robot:**
   - Follow the instructions in the `code/README.md` to start the robot.
   - Ensure that the robot is calibrated and the environment is mapped.

3. **Emergency stop:**
   - Command the robot to return to the starting point with the retrieved object.

## Contributors

- Obianuju Ochuba
- Xingjian Zhang
- Feihan Hao
- Odysseas Bouziotis
  

