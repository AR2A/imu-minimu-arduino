# ROS MiniImu Calibration and Sensor Fusion

## Overview
This is a [ROS] package developed to calibrate and fuse the orientation data provided by an Polulu MiniImu v9.  

The ROS MiniImu Calibration and Sensor Fusion Packages are tested under ROS Indigo and Ubuntu 14.04.

**Affiliation: Advancements for Robotics in Rescue Applications, AR2A**

##Travis Build Status
[![Build Status](https://travis-ci.org/AR2A/imu-minimu-arduino.svg?branch=master)](https://travis-ci.org/AR2A/imu-minimu-arduino)

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

### Downloading the firmware to the arduino

In order for the download to work modify the file "avr_imu/firmware/CMakeLists.txt" to suit your setup.

        cmake_minimum_required(VERSION 2.8.3)
        
        include_directories(ros_lib arduino_libs)
        
        generate_arduino_firmware(avr_imu
        	SRCS avr_imu.cpp arduino_libs/L3G/L3G.cpp arduino_libs/LSM303/LSM303.cpp ros_lib/time.cpp
        	BOARD mega2560
        	PORT /dev/ttyACM0
        )

Most likely the variables BOARD and PORT have to be modified.

Afterwards you download the firmware by issuing the following commands inside your catkin workspace.

        catkin_make avr_imu_firmware_avr_imu
        catkin_make avr_imu_firmware_avr_imu-upload

### Visualizing data with [rviz]

For debugging purposes it can be useful to visualize the orientation within [rviz]. To do so install the imu_tools package which contains the [rviz_imu_plugin](http://wiki.ros.org/rviz_imu_plugin) package.

        sudo apt-get install ros-indigo-imu-tools

## Nodes

### Node: avr_imu

This is the rosserial node running on the arduino board. The node reads the data from the imu over the spi interface and issues a message containing this data in a 50 Hz cycle.

#### Subscribed Topics

*None*

#### Published Topics

* **`/imu/data_raw`** ([sensor_msgs/Imu])

    The raw register data of the imu (accelerometer and gyroscope).
	
* **`/imu/magnetic_field`** ([sensor_msgs/MagneticField])

	The raw register data of the imu (magnetometer).

#### Services

*None*

#### Parameters

*None*

### Node: calib_imu(experimental)

This node executes a step by step calibration process to compensate the alignement and scaling of the imu data (as well as hard and soft ironing effects on the magnetometer data). When the calibration finishes the calculated data is stored inside the ros execution path (typical ~/.ros).

#### Subscribed Topics

* **`/imu/data_raw`** ([sensor_msgs/Imu])

    The raw register data of the imu (accelerometer and gyroscope).
	
* **`/imu/magnetic_field`** ([sensor_msgs/MagneticField])

	The raw register data of the imu (magnetometer).
    
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

* **`/imu/data_raw`** ([sensor_msgs/Imu])

    The raw register data of the imu (accelerometer and gyroscope).
	
* **`/imu/magnetic_field`** ([sensor_msgs/MagneticField])

	The raw register data of the imu (magnetometer).

#### Published Topics

* **`/imu/data`** ([sensor_msgs/Imu])

    The fused orientation data.

#### Services

*None*

#### Parameters

*None*


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/AR2A/imu-minimu-arduino/issues).

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[avr_imu/Imu]: https://github.com/AR2A/imu-minimu-arduino/blob/master/avr_imu/msg/Imu.msg
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[sensor_msgs/MagneticField]: http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
[std_msgs/Empty]: http://docs.ros.org/api/std_msgs/html/msg/Empty.html
