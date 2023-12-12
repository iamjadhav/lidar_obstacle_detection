## Lidar Obstacle Detection

---

## Overview

Sensor fusion is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

In this project I have implemented a Lidar Obstacle Detection module, which detects and tracks obstacles on the road using Lidar point clouds, 3D RANSAC Planar Segmentation, 3D Kd-Tree and Euclidean Clustering algorithms also using several Point Cloud Library methods such as Filtering, Extracting Indices, Crop Box, etc.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

## Technology Used

* Ubuntu 16.04 LTS / Windows 11
* C++11
* Eigen
* CMake Build System
* Doxygen
* PCL - v1.11.1
* gcc v5.5

## Demos and Overviews

Demo Obstacle Detection:

![lidarObstacleDetection](https://github.com/iamjadhav/lidar_obstacle_detection/assets/35925489/f423bf00-6e1d-4652-9dbe-25a438559f8d)

Demo Tracking a Bicyclist:

![bicyclist_tracking](https://github.com/iamjadhav/lidar_obstacle_detection/assets/35925489/7f2cff45-2493-4740-abc1-23d7c1466ba3)


## Set of Assumptions 

- All obstacle movement is in the X-Y plane

## Known Issues/Bugs 

- Some Obstacle bounding boxes merge into each other forming a huge box when two cars drive by for an instant
- The pole is not detected in a few frames

## Dependencies

- Install PCL 1.11.1 on Windows using this link. Refer [PCL(Windows 11)](https://github.com/PointCloudLibrary/pcl/issues/4462)

## Ubuntu Installation

1. Clone this github repo:

   ```sh
   cd ~
   git clone --recursive https://github.com/iamjadhav/lidar_obstacle_detection
   ```

2.  Edit [CMakeLists.txt](https://github.com/udacity/lidar_obstacle_detection/blob/master/CMakeLists.txt) as follows:

   ```cmake
   cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
   
   add_definitions(-std=c++14)
   
   set(CXX_FLAGS "-Wall")
   set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
   
   project(playback)
   
   find_package(PCL 1.11 REQUIRED)
   
   include_directories(${PCL_INCLUDE_DIRS})
   link_directories(${PCL_LIBRARY_DIRS})
   add_definitions(${PCL_DEFINITIONS})
   list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
   
   
   add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
   target_link_libraries (environment ${PCL_LIBRARIES})
   ```

3. Execute the following commands in a terminal

   ```shell
   sudo apt install libpcl-dev
   cd ~/lidar_obstacle_detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

   This should install the latest version of PCL. You should be able to do all the project with this setup.
   
**Note** The library version of PCL being distributed by the apt repository for 18.04 and 20.04 are both older than v1.11. The following links have the information regarding the versions-

[Bionic 18.04](https://www.ubuntuupdates.org/package/core/bionic/universe/updates/libpcl-dev)
[Focal 20.04](https://www.ubuntuupdates.org/package/core/focal/universe/base/libpcl-dev)

You can either build PCL from source (for v1.11) or use the older version.

## How to build and run on Visual Studio 2019

```
git clone --recursive https://github.com/iamjadhav/lidar_obstacle_detection
cd lidar_obstacle_detection
mkdir build
cd build
cmake ..
Run environment.cpp
```

## Links

Final Output and Overview --> 

https://github.com/iamjadhav/lidar_obstacle_detection/assets/35925489/98bf36d1-0bb2-4a75-8ed7-48aab2db778f

![lidarObstacleDetection_bicyclist_tracking](https://github.com/iamjadhav/lidar_obstacle_detection/assets/35925489/2c439fa0-9301-4a2b-95e4-0744e6051ec7)

PCL Documentation --> [Link](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html)
