/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     main.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/
#include <string>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <armadillo>

#include "Sensor3DCalibration.h"
#include "SensorFusion.h"

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

 /**
 * @brief
 * @param[in] msg_imu
 * @param[in] msg_mag
 *
 * @return
 */
void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msgImu, const sensor_msgs::MagneticField::ConstPtr& msgMag);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

Sensor3DCalibration acc_cal(PATH_TO_CALIBRATION,"acc");
Sensor3DCalibration ang_cal(PATH_TO_CALIBRATION,"ang");
Sensor3DCalibration mag_cal(PATH_TO_CALIBRATION,"mag");

arma::vec acc_vec(3,arma::fill::zeros);
arma::vec ang_vec(3,arma::fill::zeros);
arma::vec mag_vec(3,arma::fill::zeros);

SensorFusion::FusedData data;

SensorFusion kalman(PATH_TO_CALIBRATION,1/50.0);

ros::Publisher imu_pub;

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

/**
 * @brief
 * @param[in] argc
 * @param[in] argv
 *
 * @return
 */
int main(int argc, char ** argv) {
    ros::init(argc,argv,"process_imu_data");

    ros::NodeHandle nh;
    imu_pub=nh.advertise<sensor_msgs::Imu>(ros::names::resolve("imu") + "/data", 5);

    message_filters::Subscriber<sensor_msgs::Imu> * sub_imu_data = new message_filters::Subscriber<sensor_msgs::Imu>(nh,ros::names::resolve("imu") + "/data_raw", QUEUE_LENGTH);
    message_filters::Subscriber<sensor_msgs::MagneticField> * sub_mag_data = new message_filters::Subscriber<sensor_msgs::MagneticField>(nh,ros::names::resolve("imu") + "/magnetic_field", QUEUE_LENGTH);

    Synchronizer * sync = new Synchronizer(SyncPolicy(QUEUE_LENGTH),*sub_imu_data,*sub_mag_data);
    sync->registerCallback(imuDataArrived);

    ros::spin();

    return 0;
}

void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::MagneticField::ConstPtr& msg_mag) {
    sensor_msgs::Imu msg_out;
    tf::Quaternion q;

    msg_out.header.stamp=ros::Time::now();
    msg_out.header.frame_id = msg_imu->header.frame_id;

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


    //Calibrate each Vector (Sensitivity matrix contains unit conversion)
    acc_vec=acc_cal(acc_vec);
    mag_vec=mag_cal(mag_vec);
    ang_vec=ang_cal(ang_vec);

    data=kalman(ang_vec,acc_vec,mag_vec);

    //Rotation around XYZ (Roll/Pitch/Yaw)
    q.setRPY(data.angles(0),data.angles(1), data.angles(2));
    tf::quaternionTFToMsg(q,msg_out.orientation);

    for(size_t i=0; i<3; i++) {
        for(size_t ii=0; ii<3; ii++) {
            msg_out.orientation_covariance[i*3+ii] = data.covariance(i,ii);
        }
    }

    msg_out.angular_velocity.x = ang_vec(0);
    msg_out.linear_acceleration.x = acc_vec(0);
    msg_out.angular_velocity.y = ang_vec(1);
    msg_out.linear_acceleration.y = acc_vec(1);
    msg_out.angular_velocity.z = ang_vec(2);
    msg_out.linear_acceleration.z = acc_vec(2);

    imu_pub.publish(msg_out);
    ros::spinOnce();
}
