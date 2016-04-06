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

#include "CalibrationGenerator.h"
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
static size_t const QUEUE_LENGTH=5;

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

CalibrationGenerator cal_gen(1.0,9.81,1.0);

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

int main(int argc, char ** argv) {
    ros::init(argc,argv,"calib_imu");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Imu> * sub_imu_data = new message_filters::Subscriber<sensor_msgs::Imu>(nh,ros::names::resolve("imu") + "/data_raw", QUEUE_LENGTH);
    message_filters::Subscriber<sensor_msgs::MagneticField> * sub_mag_data = new message_filters::Subscriber<sensor_msgs::MagneticField>(nh,ros::names::resolve("imu") + "/magnetic_field", QUEUE_LENGTH);

    Synchronizer * sync = new Synchronizer(SyncPolicy(QUEUE_LENGTH),*sub_imu_data,*sub_mag_data);
    sync->registerCallback(imuDataArrived);

    ros::spin();

    return EXIT_SUCCESS;
}

void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::MagneticField::ConstPtr& msg_mag) {
    static bool calibrated = false;

    //Get data into vectors
    acc_vec(0) = msg_imu->linear_acceleration.x;
    acc_vec(1) = msg_imu->linear_acceleration.y;
    acc_vec(2) = msg_imu->linear_acceleration.z;
    ang_vec(0) = msg_imu->angular_velocity.x;
    ang_vec(1) = msg_imu->angular_velocity.y;
    ang_vec(2) = msg_imu->angular_velocity.z;
    mag_vec(0) = msg_mag->magnetic_field.x;
    mag_vec(1) = msg_mag->magnetic_field.y;
    mag_vec(2) = msg_mag->magnetic_field.z;

    if(!calibrated) {
        cal_gen.CalibrationStep(mag_vec,acc_vec,ang_vec);

        if(cal_gen.isFinished()) {
            cal_gen.InitialiseCalibrationObjectMag(mag_cal);
            cal_gen.InitialiseCalibrationObjectAcc(acc_cal);
            cal_gen.InitialiseCalibrationObjectGyr(ang_cal);
            calibrated = true;
        }
    }
}
