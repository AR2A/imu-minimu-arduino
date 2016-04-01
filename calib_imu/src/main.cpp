/**
 * \author   Christian Breitwieser, BSc
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     main.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <string>

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <tf/tf.h>

#include <avr_imu/Imu.h>

#include <armadillo>

#include "CalibrationGeneration.h"
#include "../../process_imu_data/src/Sensor3DCalibration.h"

using namespace std;

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static string const PATH_TO_CALIBRATION = "./calibration";

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void imuDataArrived(const avr_imu::Imu::ConstPtr& msg);
void advanceStatemachine(const std_msgs::Empty& msg);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

Sensor3DCalibration acc_cal(PATH_TO_CALIBRATION, "acc");
Sensor3DCalibration ang_cal(PATH_TO_CALIBRATION, "ang");
Sensor3DCalibration mag_cal(PATH_TO_CALIBRATION, "mag");

arma::vec acc_vec(3,arma::fill::zeros);
arma::vec ang_vec(3,arma::fill::zeros);
arma::vec mag_vec(3,arma::fill::zeros);

CalibrationGeneration calGen(1.0,9.81,1.0);
ros::Publisher calib_finished_pub;

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"process_imu_data");

  ros::NodeHandle nh;
  
  calib_finished_pub=nh.advertise<std_msgs::Empty>("processed_imu_data", 100);

  ros::Subscriber sub = nh.subscribe ("raw_imu_data", 100, imuDataArrived);
  ros::Subscriber sub1 = nh.subscribe ("imuCalibProceed", 100, advanceStatemachine);
  
  ros::spin();

  return EXIT_SUCCESS;
}

void advanceStatemachine(const std_msgs::Empty& msg)
{
  calGen.progressStep();
}

void imuDataArrived(const avr_imu::Imu::ConstPtr& msg)
{

  static bool calibrated = false;

  //Get data into vectors
  acc_vec(0) = msg->linear_acceleration[0];
  acc_vec(1) = msg->linear_acceleration[1];
  acc_vec(2) = msg->linear_acceleration[2];
  ang_vec(0) = msg->angular_velocity[0];
  ang_vec(1) = msg->angular_velocity[1];
  ang_vec(2) = msg->angular_velocity[2];
  mag_vec(0) = msg->magnetic_field[0];
  mag_vec(1) = msg->magnetic_field[1];
  mag_vec(2) = msg->magnetic_field[2];

  if(!calibrated)
  {
    calGen.CalibrationStep(mag_vec,acc_vec,ang_vec);
    
    if(calGen.isFinished())
    {
      std_msgs::Empty msgOut;
      calGen.InitialiseCalibrationObjectMag(mag_cal);
      calGen.InitialiseCalibrationObjectAcc(acc_cal);
      calGen.InitialiseCalibrationObjectGyr(ang_cal);
      calibrated = true;
      calib_finished_pub.publish(msgOut);
    }
  }
}
