# ROS MiniImu Calibration and Sensor Fusion

## Overview
This is a [ROS] package developed to calibrate and fuse the orientation data provided by an Polulu MiniImu v9.  

The ROS MiniImu Calibration and Sensor Fusion Packages are tested under ROS Indigo and Ubuntu 14.04.

**Affiliation: Advancements for Robotics in Rescue Applications, AR2A**

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the ROS MiniImu Calibration and Sensor Fusion depends on following software:

- [arduino toolchain](https://www.arduino.cc) (simple programming environment for education pcbs)

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

To use the packages in your project, clone the latest version of this repository to the source folder of your catkin workspace.
Make sure that all dependencies listed above are installed before executing catkin_make.

        cd ~/catkin_ws/src
        git clone https://github.com/AR2A/imu-minimu-arduino.git
        cd ..
        catkin_make

## Basic Usage

## Nodes

### Node: avr_imu

This is the rosserial node running on the arduino board. The node reads the data from the imu over the spi interface and issues a message containing this data in a 50 Hz cycle.

#### Subscribed Topics

*None*

#### Published Topics

* **`raw_imu_data`** ([avr_imu/Imu])

    The raw register data of the imu.

#### Services

*None*

#### Parameters

*None*

### Node: calib_imu(experimental)

This node executes a step by step calibration process to compensate the alignement and scaling of the imu data (as well as hard and soft ironing effects on the magnetometer data). When the calibration finishes the calculated data is stored inside the ros execution path (typical ~/.ros).

#### Subscribed Topics

* **`raw_imu_data`** ([avr_imu/Imu])

    The raw register data of the imu.
    
* **`imuCalibProceed`** ([std_msgs/Empty])

    When received the calibration process proceeds.
    
#### Published Topics

*None*

#### Services

*None*

#### Parameters

*None*

### Node: process_imu_data

This node processes the senor fusion with an extended kalman filter 

The node is roughly designed after the findings of the following paper: 

> David Jurman, Marko Jankovec, Roman Kamnik, Marko Topic, "[Calibration and data fusion solution for the miniature
attitude and heading reference system](http://www.sciencedirect.com/science/article/pii/S0924424707003834)", in Sensors and Actuators A: Physical, Volume 138, 2007.



#### Subscribed Topics

* **`raw_imu_data`** ([avr_imu/Imu])

    The raw register data of the imu.

#### Published Topics

* **`processed_imu_data`** ([sensor_msgs/Imu])

    The fused orientation data.

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
