cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR} arduino_libs)

generate_arduino_firmware(avr_imu
	SRCS avr_imu.cpp arduino_libs/L3G/L3G.cpp arduino_libs/LSM303/LSM303.cpp ${ROS_LIB_DIR}/time.cpp
	BOARD mega2560
	PORT /dev/ttyACM0
)
