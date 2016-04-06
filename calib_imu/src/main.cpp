/**
 * \author   CB
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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <armadillo>

#include "CalibrationGeneration.h"
#include "../../process_imu_data/src/Sensor3DCalibration.h"

using namespace std;

/**************************************************************************************
 * TYPES
 **************************************************************************************/

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu ,sensor_msgs::MagneticField> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static string const PATH_TO_CALIBRATION = "./calibration";
static size_t queue_length=5;

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msgImu, const sensor_msgs::MagneticField::ConstPtr& msgMag);

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

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"calib_imu");

  ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Imu> * subImuData = new message_filters::Subscriber<sensor_msgs::Imu>(nh,ros::names::resolve("imu") + "/data_raw", queue_length);
	message_filters::Subscriber<sensor_msgs::MagneticField> * subMagData = new message_filters::Subscriber<sensor_msgs::MagneticField>(nh,ros::names::resolve("imu") + "/magnetic_field", queue_length);

	Synchronizer * sync = new Synchronizer(SyncPolicy(queue_length),*subImuData,*subMagData);
	sync->registerCallback(imuDataArrived);
	  
  ros::spin();

  return EXIT_SUCCESS;
}

void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msgImu, const sensor_msgs::MagneticField::ConstPtr& msgMag)
{

  static bool calibrated = false;

  //Get data into vectors
  acc_vec(0) = msgImu->linear_acceleration.x;
  acc_vec(1) = msgImu->linear_acceleration.y;
  acc_vec(2) = msgImu->linear_acceleration.z;
  ang_vec(0) = msgImu->angular_velocity.x;
  ang_vec(1) = msgImu->angular_velocity.y;
  ang_vec(2) = msgImu->angular_velocity.z;
  mag_vec(0) = msgMag->magnetic_field.x;
  mag_vec(1) = msgMag->magnetic_field.y;
  mag_vec(2) = msgMag->magnetic_field.z;

  if(!calibrated)
  {
    calGen.CalibrationStep(mag_vec,acc_vec,ang_vec);
    
    if(calGen.isFinished())
    {
      calGen.InitialiseCalibrationObjectMag(mag_cal);
      calGen.InitialiseCalibrationObjectAcc(acc_cal);
      calGen.InitialiseCalibrationObjectGyr(ang_cal);
      calibrated = true;
    }
  }
}
