/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     main.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <iostream>
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

/**
 * @brief Path where the calibration matrices are stored.
 * @todo Introduce a ros parameter to make this value configurable
 */
static string const PATH_TO_CALIBRATION = "./calibration";

static size_t const QUEUE_LENGTH=5; /**< Length of the queues attached to the message buffers of subsrcibed messages*/

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

/**
 * @brief Callback for received imu data.
 * Gets called every time new imu data arrives - the data is split in two messages
 * (one for gyro and accelerometer as well as one for magnetometer readings)
 *
 * @param[in] msg_imu Gyro/accelerometer readings
 * @param[in] msg_mag Magnetometer readings
 */
void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::MagneticField::ConstPtr& msg_mag);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

Sensor3DCalibration acc_cal(PATH_TO_CALIBRATION, "acc"); /**< Container used for calibration data for the accelerometer */
Sensor3DCalibration ang_cal(PATH_TO_CALIBRATION, "ang"); /**< Container used for calibration data for the gyro */
Sensor3DCalibration mag_cal(PATH_TO_CALIBRATION, "mag"); /**< Container used for calibration data for the magnetometer */

arma::vec acc_vec(3,arma::fill::zeros); /**< Vector representation of one accelerometer reading */
arma::vec ang_vec(3,arma::fill::zeros); /**< Vector representation of one gyro reading */
arma::vec mag_vec(3,arma::fill::zeros); /**< Vector representation of one magnetometer reading */

/**
 * @brief Calculates the calibration data for the three sensors of the imu.
 */
CalibrationGenerator cal_gen(
    /*   Magnetometer zero position amplitude: */ 1.00, /* normalized */
    /* Accelerometer zero position  amplitude: */ 9.81, /* m/s^2 */
    /*          Gyro zero positiuon amplitude: */ 1.00);/* normalized */

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

int main(int argc, char ** argv) {
    ros::init(argc,argv,"calib_imu");
    ros::NodeHandle nh;

    //Create the subscribers for both imu porovided messages
    message_filters::Subscriber<sensor_msgs::Imu> * sub_imu_data =
        new message_filters::Subscriber<sensor_msgs::Imu>(nh,ros::names::resolve("imu") + "/data_raw", QUEUE_LENGTH);

    message_filters::Subscriber<sensor_msgs::MagneticField> * sub_mag_data =
        new message_filters::Subscriber<sensor_msgs::MagneticField>(nh,ros::names::resolve("imu") + "/magnetic_field", QUEUE_LENGTH);

    //Synchronize both messages to the same timebase (they should be sent by the imu at the 'same' time.)
    Synchronizer * sync = new Synchronizer(SyncPolicy(QUEUE_LENGTH),*sub_imu_data,*sub_mag_data);

    //Register a callback which is called every time synchronized readings are available
    sync->registerCallback(imuDataArrived);

    //Start the ros thread (single threaded)
    ros::spin();

    return EXIT_SUCCESS;
}

void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::MagneticField::ConstPtr& msg_mag) {

    //Convert the ros vector3 datatypes to a armadillo vector (the internally used math library)
    acc_vec(0) = msg_imu->linear_acceleration.x;
    acc_vec(1) = msg_imu->linear_acceleration.y;
    acc_vec(2) = msg_imu->linear_acceleration.z;
    ang_vec(0) = msg_imu->angular_velocity.x;
    ang_vec(1) = msg_imu->angular_velocity.y;
    ang_vec(2) = msg_imu->angular_velocity.z;
    mag_vec(0) = msg_mag->magnetic_field.x;
    mag_vec(1) = msg_mag->magnetic_field.y;
    mag_vec(2) = msg_mag->magnetic_field.z;

    //Process the current dataset (calculate one step of the calibration)
    cal_gen.CalibrationStep(mag_vec,acc_vec,ang_vec);

    //When the calibration is finished store the calibration data to the working directory
    //and finish. (Storing of the data is done by the Sensor3DCalibration objects).
    if(cal_gen.isFinished()) {
        cal_gen.InitialiseCalibrationObjectMag(mag_cal);
        cal_gen.InitialiseCalibrationObjectAcc(acc_cal);
        cal_gen.InitialiseCalibrationObjectGyr(ang_cal);
        cout << "Calibration finished!" << endl;
        ros::shutdown();
    }
}
