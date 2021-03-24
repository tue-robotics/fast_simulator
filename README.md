# Fast Simulator

[![CI](https://github.com/tue-robotics/fast_simulator/workflows/CI/badge.svg)](https://github.com/tue-robotics/fast_simulator/actions)

## Introduction

The fast_simulator is a robot simulator for ROS. It simulates simple kinematic movements, as well as laser and rgbd sensor data.

## Installation

We assume you have successfully installed ROS and set-up a Catkin workspace. Check out the following packages in your workspace:

```bash
cd <your_catkin_workspace>/src

git clone https://github.com/tue-robotics/fast_simulator.git
git clone https://github.com/tue-robotics/fast_simulator_data.git
git clone https://github.com/tue-robotics/virtual_cam.git
git clone https://github.com/tue-robotics/geolib2.git
git clone https://github.com/tue-robotics/code_profiler.git
git clone https://github.com/tue-robotics/tue_filesystem.git
git clone https://github.com/tue-robotics/ed_object_models.git
git clone https://github.com/tue-robotics/tue_config.git
git clone https://github.com/tue-robotics/tue_msgs.git
```

You will also need the following system dependencies:

```bash
sudo apt-get install ros-$ROS_DISTRO-geometry-msgs ros-$ROS_DISTRO-kdl-parser yaml-cpp ros-$ROS_DISTRO-roslib ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-tf-conversions libassimp-dev ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-common-msgs ros-$ROS_DISTRO-ros-comm ros-$ROS_DISTRO-message-generation ros-$ROS_DISTRO-stereo-msgs ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-opencv2 ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-message-runtime ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-message-filters ros-$ROS_DISTRO-roscpp ros-$ROS_DISTRO-image-geometry
```

This should be sufficient to successfully compile the simulator:

```bash
cd <your_catkin_workspace>
catkin build/catkin_make
```
