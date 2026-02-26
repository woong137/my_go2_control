# Introduction

This package allows you to use the Unitree Go2 API by publishing ROS2 topics and calling ROS2 services.

## Configuration

### System requirements

- Ubuntu 22.04
- ROS2 Humble: https://docs.ros.org/en/humble/Installation.html
- unitree_ros2 package: https://github.com/unitreerobotics/unitree_ros2
- unitree sdk2 package: https://github.com/unitreerobotics/unitree_sdk2

### Install

1. Clone this repository into unitree_ros2/example/src/src folder

```bash
cd ~/unitree_ros2/example/src/src
git clone https://github.com/woong137/my_go2_control.git
```

2. modify CMakeLists.txt

```bash
cp ~/unitree_ros2/example/src/src/my_go2_control/Cmake_backup.txt ~/unitree_ros2/example/src/src/CMakeLists.txt
```

3. build

```bash
cd ~/unitree_ros2/example/
colcon build
```

### run

```bash
source ~/.bashrc
ros2 run unitree_ros2_example my_go2_control
```