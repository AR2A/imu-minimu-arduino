#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <L3G/L3G.h>
#include <LSM303/LSM303.h>
#include <Arduino.h>
#include <Wire.h>

ros::NodeHandle nh;
L3G gyro;
LSM303 compass;

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
ros::Publisher imu_pub("imu/data_raw", &imu_msg);
ros::Publisher mag_pub("imu/magnetic_field", &mag_msg);
long timer=0;   //general purpuse timer

 void setup()
{

	Wire.begin();
	delay(1500);
	gyro.init();
	gyro.writeReg(L3G_CTRL_REG4, 0x00); // 245 dps scale
	gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
	//8.75 mdps/LSB
	compass.init();
	compass.enableDefault();
	compass.writeReg(LSM303::CTRL2, 0x08); // 4 g scale: AFS = 001
	//0.122 mg/LSB
	compass.writeReg(LSM303::CTRL5, 0x10); // Magnetometer Low Resolution 50 Hz
	//Magnetometer 4 gauss scale : 0.16mgauss/LSB
	timer=millis();
	imu_msg.header.frame_id="base_imu_link";
	nh.initNode();
	nh.advertise(imu_pub);
	nh.advertise(mag_pub);

}

 void loop()
 {
	if((millis()-timer)>=20)  // Main loop runs at 50Hz
	{
		    timer=millis();
			gyro.read();
			compass.readAcc();
			compass.readMag();
			imu_msg.angular_velocity.x=gyro.g.x;
			imu_msg.angular_velocity.y=gyro.g.y;
			imu_msg.angular_velocity.z=gyro.g.z;
			imu_msg.linear_acceleration.x=compass.a.x;
			imu_msg.linear_acceleration.y=compass.a.y;
			imu_msg.linear_acceleration.z=compass.a.z;
			mag_msg.magnetic_field.x=compass.m.x;
			mag_msg.magnetic_field.y=compass.m.y;
			mag_msg.magnetic_field.z=compass.m.z;
			imu_pub.publish( &imu_msg );
			mag_pub.publish( &mag_msg);
			nh.spinOnce();	
	}
	nh.spinOnce();
 }
