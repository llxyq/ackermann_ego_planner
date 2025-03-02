# Overview

基于阿克曼仿真模型运行EGO-Planner导航。

# Prerequisites

**环境要求**：Ubuntu 64-bit 20.04. ROS Noetic

```bash
sudo apt install libyaml-cpp-dev
sudo apt install libeigen3-dev
sudo apt install liblcm-dev
sudo apt install libglm-dev
sudo apt-get install libarmadillo-dev

sudo apt-get install ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher ros-noetic-controller-manager ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-ackermann-msgs ros-noetic-velodyne-* ros-noetic-teleop-twist-keyboard
```

# Build

```bash
git clone https://github.com/llxyq/ackermann_ego_planner.git
cd ackermann_ego_planner
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```

# Usage

1.加载仿真环境

```bash
source devel/setup.bash
roslaunch ackermann_vehicle_gazebo ackermann_vehicle_base.launch 
```

2.启动导航节点

```bash
source devel/setup.bash
roslaunch ackermann_vehicle_navigation path_follower.launch
```

3.启动ego_planner，开始导航

```bash
source devel/setup.bash
roslaunch ego_planner run_in_sim.launch 2>/dev/null
```

使用rviz中的 '2D Nav Goal' 发布目标点开始导航。

# Acknowledgements

-  [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) 
- [ackermann_vehicle](https://github.com/hdh7485/ackermann_vehicle)

