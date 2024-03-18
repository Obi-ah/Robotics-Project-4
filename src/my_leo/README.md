## Introduction
This repository contains launch files and configurations for simulating and controlling the Leo rover within a Gazebo environment using ROS 2.

## Prerequisites
- ROS 2 humble
- Gazebo6 installed
- Python 3.6 or later
- Ubuntu 22.04

## Installation
1. Clone this repository into your ROS 2 workspace:

    ```bash
    git clone https://github.com/
    ```

2. Build and source your ROS 2 workspace:

    ```bash
    colcon build
    source install/setup.bash
    ```

## Usage
To launch the simulation and control setup, run the following command:

```bash
ros2 launch my_leo my_leo.launch.py
```

This command will start Gazebo along with the Leo rover simulation, RViz for visualization, and other necessary nodes for control and sensor communication.

After launch my_leo.launch.py, wair for exploration in simulation, then, new a terminal
```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "map_url: './src/my_leo/maps/sim_map'"
```
