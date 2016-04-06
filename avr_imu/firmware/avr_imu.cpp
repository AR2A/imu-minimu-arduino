/**
 * \author   CB
 * \brief    Arduino Firmware for the Pololu MinIMU-9.
 * \file     avr_imu.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Arduino.h>
#include <Wire.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <L3G/L3G.h>
#include <LSM303/LSM303.h>

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ros::NodeHandle nh; /**< The ros node handle, identifies the arduino inside the ros system */
L3G gyro; /**< Abstraction from the gyro/accelerometer register interface*/
LSM303 compass; /**< Abstraction from the magnetometer register interface*/

sensor_msgs::Imu imu_msg; /**< ros message which contains the gyro and accelerometer data*/
sensor_msgs::MagneticField mag_msg; /**< ros message which contains the magnetometer data*/
ros::Publisher imu_pub("imu/data_raw", &imu_msg); /**< advertises the imu message (namespace imu)*/
ros::Publisher mag_pub("imu/magnetic_field", &mag_msg); /**< advertises the magnetometer message (namespace imu)*/
long timer=0; /**< provides a timebase in milliseconds for the main loop*/

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

/**
 * @brief Initializes the gyro/accelerometer and the magnetometer unit of the imu.
 * 		As well as the arduino subsystem
 */
void setup() {
    Wire.begin();
    delay(1500);

    /********/
    /* GYRO */
    /********/
    gyro.init();
    gyro.writeReg(L3G_CTRL_REG4, 0x00); // 245 dps scale
    gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
    //8.75 mdps/LSB

    /****************/
    /* MAGNETOMETER */
    /****************/
    compass.init();
    compass.enableDefault();
    compass.writeReg(LSM303::CTRL2, 0x08); // 4 g scale: AFS = 001
    //0.122 mg/LSB
    compass.writeReg(LSM303::CTRL5, 0x10); // Magnetometer Low Resolution 50 Hz
    //Magnetometer 4 gauss scale : 0.16mgauss/LSB


    //ROS-TF base frame of the imu_data
    imu_msg.header.frame_id="base_imu_link";

    //Register ROS messages
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(mag_pub);

    //starting value for the timer
    timer=millis();
}

/**
 * @brief provides imu readings in a 50 Hz rate.
 *
 */
void loop() {
    if((millis()-timer)>=20) { // Main loop runs at 50Hz
        timer=millis();

        //Read data from the hardware

        gyro.read();
        compass.readAcc();
        compass.readMag();

        //Assign read data to the ros messages

        imu_msg.angular_velocity.x=gyro.g.x;
        imu_msg.angular_velocity.y=gyro.g.y;
        imu_msg.angular_velocity.z=gyro.g.z;

        imu_msg.linear_acceleration.x=compass.a.x;
        imu_msg.linear_acceleration.y=compass.a.y;
        imu_msg.linear_acceleration.z=compass.a.z;

        mag_msg.magnetic_field.x=compass.m.x;
        mag_msg.magnetic_field.y=compass.m.y;
        mag_msg.magnetic_field.z=compass.m.z;

        //Publish the data to the ros message system

        imu_pub.publish( &imu_msg );
        mag_pub.publish( &mag_msg);
        nh.spinOnce();
    }
    nh.spinOnce();
}
