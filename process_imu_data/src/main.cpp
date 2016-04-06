#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "tf/tf.h"
#include <armadillo>
#include "Sensor3DCalibration.h"
#include "SensorFusion.h"

#include <string>

using namespace std;

/**************************************************************************************
 * TYPES
 **************************************************************************************/

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu ,sensor_msgs::MagneticField> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

string const path="./calibration";
static size_t queue_length=5;


Sensor3DCalibration acc_cal(path,"acc");
Sensor3DCalibration ang_cal(path,"ang");
Sensor3DCalibration mag_cal(path,"mag");

arma::vec acc_vec(3,arma::fill::zeros);
arma::vec ang_vec(3,arma::fill::zeros);
arma::vec mag_vec(3,arma::fill::zeros);

SensorFusion::FusedData data;


SensorFusion kalman(path,1/50.0);

void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msgImu, const sensor_msgs::MagneticField::ConstPtr& msgMag);

ros::Publisher imu_pub;

int main(int argc, char ** argv)
{

ros::init(argc,argv,"process_imu_data");

ros::NodeHandle nh;
imu_pub=nh.advertise<sensor_msgs::Imu>(ros::names::resolve("imu") + "/data", 5);

	message_filters::Subscriber<sensor_msgs::Imu> * subImuData = new message_filters::Subscriber<sensor_msgs::Imu>(nh,ros::names::resolve("imu") + "/data_raw", queue_length);
	message_filters::Subscriber<sensor_msgs::MagneticField> * subMagData = new message_filters::Subscriber<sensor_msgs::MagneticField>(nh,ros::names::resolve("imu") + "/magnetic_field", queue_length);

Synchronizer * sync = new Synchronizer(SyncPolicy(queue_length),*subImuData,*subMagData);
sync->registerCallback(imuDataArrived);

ros::spin();

return 0;
}

void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msgImu, const sensor_msgs::MagneticField::ConstPtr& msgMag)
{
sensor_msgs::Imu msgOut;

  msgOut.header.stamp=ros::Time::now();//msgImu->header.stamp;
  msgOut.header.frame_id = msgImu->header.frame_id;

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


	//Calibrate each Vector (Sensitivity matrix contains unit conversion)
	acc_vec=acc_cal(acc_vec);
	mag_vec=mag_cal(mag_vec);
	ang_vec=ang_cal(ang_vec);

	data=kalman(ang_vec,acc_vec,mag_vec);
	

	tf::Quaternion q;

	//Rotation around XYZ (Roll/Pitch/Yaw)
	q.setRPY(data.angles(0),data.angles(1), data.angles(2));
	tf::quaternionTFToMsg(q,msgOut.orientation);

	for(size_t i=0;i<3;i++){
		for(size_t ii=0;ii<3;ii++){
			msgOut.orientation_covariance[i*3+ii] = data.covariance(i,ii);
		}
	}
	msgOut.angular_velocity.x = ang_vec(0);
	msgOut.linear_acceleration.x = acc_vec(0);
	msgOut.angular_velocity.y = ang_vec(1);
	msgOut.linear_acceleration.y = acc_vec(1);
	msgOut.angular_velocity.z = ang_vec(2);
	msgOut.linear_acceleration.z = acc_vec(2);
	
	
	imu_pub.publish(msgOut);
	ros::spinOnce();
	

}
