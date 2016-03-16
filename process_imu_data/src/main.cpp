#include "ros/ros.h"
#include "avr_imu/Imu.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include <armadillo>
#include "Sensor3DCalibration.h"
#include "SensorFusion.h"

#include <string>

using namespace std;

string const path="./calibration";


Sensor3DCalibration acc_cal(path,"acc");
Sensor3DCalibration ang_cal(path,"ang");
Sensor3DCalibration mag_cal(path,"mag");

arma::vec acc_vec(3,arma::fill::zeros);
arma::vec ang_vec(3,arma::fill::zeros);
arma::vec mag_vec(3,arma::fill::zeros);

SensorFusion::FusedData data;


SensorFusion kalman(path,1/50.0);

void imuDataArrived(const avr_imu::Imu::ConstPtr& msg);

ros::Publisher imu_pub;

int main(int argc, char ** argv)
{

ros::init(argc,argv,"process_imu_data");

ros::NodeHandle nh;
imu_pub=nh.advertise<sensor_msgs::Imu>("processed_imu_data", 100);

ros::Subscriber sub = nh.subscribe ("raw_imu_data", 100, imuDataArrived);



ros::spin();

return 0;
}

void imuDataArrived(const avr_imu::Imu::ConstPtr& msg){
	//Get data into vectors
	acc_vec(0)=msg->linear_acceleration[0];
	acc_vec(1)=msg->linear_acceleration[1];
	acc_vec(2)=msg->linear_acceleration[2];
	ang_vec(0)=msg->angular_velocity[0];
	ang_vec(1)=msg->angular_velocity[1];
	ang_vec(2)=msg->angular_velocity[2];
	mag_vec(0)=msg->magnetic_field[0];
	mag_vec(1)=msg->magnetic_field[1];
	mag_vec(2)=msg->magnetic_field[2];


	//Calibrate each Vector (Sensitivity matrix contains unit conversion)
	acc_vec=acc_cal(acc_vec);
	mag_vec=mag_cal(mag_vec);
	ang_vec=ang_cal(ang_vec);

	data=kalman(ang_vec,acc_vec,mag_vec);
	
	sensor_msgs::Imu msgOut;
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
	msgOut.header.stamp=ros::Time::now();
	msgOut.header.frame_id = "base_imu_link";
	imu_pub.publish(msgOut);
	ros::spinOnce();
	//data.angles*=(180.0/(3.14159265));
	
	//data.angles.print("Angles: ");
	

	//Third Step: Compensation of tilt of magnetometer

	//Fourth Step: Compensation of dynamic acceleration of accelerometer

	//Fifth Step: Calculation of Roll/Pitch out of the g vector of the accelerometer

	//Sixth Step: Calculation of Yaw out of the alignement to magnetic north of the magnetometer

	//Seventh Step: Kalman Prediction with Gyro Data and Previous State estimation

	//Eight Step: Kalman Update and Correction with Calculated Roll/Pitch/Yaw

	//Ninth Step: Compensation of static acceleration of the accelerometer

	//Tenth Step: Kalman Prediction with remaining dynamic acceleration

	//Elevent Step: Kalman Update and Correction with Velocity Published by MotorDrivers

	//Twelvth Step: Publish of current Orientation as tf::transform (with regard to fixed position relative to the robot frame)

	//Thirteenth Step: Publish of current Velocity
	

}
