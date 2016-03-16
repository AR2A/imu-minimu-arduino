#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "avr_imu/Imu.h"
#include "tf/tf.h"
#include <armadillo>
#include "../../process_imu_data/src/Sensor3DCalibration.h"
#include "CalibrationGeneration.h"

#include <string>

using namespace std;

string const path="./calibration";


Sensor3DCalibration acc_cal(path,"acc");
Sensor3DCalibration ang_cal(path,"ang");
Sensor3DCalibration mag_cal(path,"mag");

arma::vec acc_vec(3,arma::fill::zeros);
arma::vec ang_vec(3,arma::fill::zeros);
arma::vec mag_vec(3,arma::fill::zeros);


CalibrationGeneration calGen(1.0,9.81,1.0);
ros::Publisher calib_finished_pub;

void imuDataArrived(const avr_imu::Imu::ConstPtr& msg);

void advanceStatemachine(const std_msgs::Empty& msg);


int main(int argc, char ** argv)
{

ros::init(argc,argv,"process_imu_data");

ros::NodeHandle nh;
calib_finished_pub=nh.advertise<std_msgs::Empty>("processed_imu_data", 100);

ros::Subscriber sub = nh.subscribe ("raw_imu_data", 100, imuDataArrived);
ros::Subscriber sub1 = nh.subscribe ("imuCalibProceed", 100, advanceStatemachine);
ros::spin();

return 0;
}

void advanceStatemachine(const std_msgs::Empty& msg){
	calGen.progressStep();
}

void imuDataArrived(const avr_imu::Imu::ConstPtr& msg){
	static bool calibrated=false;
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

	if(!calibrated){
		calGen.CalibrationStep(mag_vec,acc_vec,ang_vec);
		if(calGen.isFinished()){
			std_msgs::Empty msgOut;
			calGen.InitialiseCalibrationObjectMag(mag_cal);
			calGen.InitialiseCalibrationObjectAcc(acc_cal);
			calGen.InitialiseCalibrationObjectGyr(ang_cal);
			calibrated=true;
			calib_finished_pub.publish(msgOut);
		}
	}
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
