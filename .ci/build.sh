#!/usr/bin/env bash

source /opt/ros/indigo/setup.bash
mkdir -p ./catkin_ws/src
cd ./catkin_ws/src
catkin_init_workspace
cp -R ../../avr_imu ./avr_imu
cp -R ../../calib_imu ./calib_imu
cp -R ../../process_imu_data ./process_imu_data
cd ..
catkin_make