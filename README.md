# ROS MiniImu Calibration and Sensor Fusion

## Overview
This is a [ROS] package developed to calibrate and fuse the orientation data provided by an Polulu MiniImu v9.  

The ROS MiniImu Calibration and Sensor Fusion Packages are tested under ROS Indigo and Ubuntu 14.04.

**Affiliation: Advancements for Robotics in Rescue Applications, AR2A**

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the ROS MiniImu Calibration and Sensor Fusion depends on following software:

- [rosserial](https://github.com/ros-drivers/rosserial) (wrapper to transmit ros messages over a serial port)

        sudo apt-get install ros-indigo-rosserial-arduino
        sudo apt-get install ros-indigo-rosserial

- [Boost](http://www.boost.org) (general purpose c++ library),

        sudo apt-get install libboost-dev
  
- [armadillo](http://arma.sourceforge.net/) (c++ linear algebra library),

        sudo apt-get install liblapack-dev
        sudo apt-get install libblas-dev
        sudo apt-get install libarmadillo-dev

### Building

## Basic Usage

## Nodes

### Node: avr_imu

#### Subscribed Topics

*None*

#### Published Topics

#### Services

*None*

#### Parameters

*None*

### Node: calib_imu

#### Subscribed Topics

#### Published Topics

*None*

#### Services

*None*

#### Parameters

*None*

### Node: process_imu_data

#### Subscribed Topics

#### Published Topics

#### Services

*None*

#### Parameters

*None*


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AR2A/imu-minimu-arduino/issues).

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[grid_map_msg/GridMap]: https://github.com/ethz-asl/grid_map/blob/master/grid_map_msg/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[grid_map_msg/GetGridMap]: https://github.com/ethz-asl/grid_map/blob/master/grid_map_msg/srv/GetGridMap.srv
