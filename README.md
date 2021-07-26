# gopro_ros

This repository contains code for parsing GoPro telemetry metadata to obtain GoPro images with synchronized IMU measurements. The GoPro visual-inertial data can then be saved in [rosbag](http://wiki.ros.org/rosbag) or [Euroc](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) format. Thus, effectively paving the way for visual-inertial odometry/SLAM for GoPro cameras.

# Installation

Tested on Ubuntu 18.04 (ros-melodic) & 20.04 (ros-noetic). The install instructions are for Ubuntu 18.04.

## Prerequisites

- ros-melodic-desktop-full
- [OpenCV](https://github.com/opencv/opencv) >= 3.2
- [FFmpeg](http://ffmpeg.org/)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Install Dependencies

- Install ROS using [this guide](http://wiki.ros.org/ROS/Installation)

- System installation of OpenCV should work

```bash
sudo apt install libopencv-dev
```

- Install FFmpeg

```bash
sudo apt install ffmpeg
```

- Install Eigen3

```bash
sudo apt install libeigen3-dev
```

## Install gopro_ros

Before proceeding, ensure all dependencies are installed. To install gopro_ros

```bash
mkdir -p ~/gopro_ws/src
cd gopro_ws/src
git clone https://github.com/joshi-bharat/gopro_ros.git
cd ~/gopro_ws/
catkin_make
source ~/gopro_ws/devel/setup.bash
# add this to ~/.bashrc to make this permanent 
```

# Usage

## Save to rosbag


## Save as Euroc dataset
