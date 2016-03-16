#include <ros.h>
#include <avr_imu/Imu.h>
#include <L3G/L3G.h>
#include <LSM303/LSM303.h>
#include <Arduino.h>
#include <Wire.h>

ros::NodeHandle nh;
L3G gyro;
LSM303 compass;

avr_imu::Imu imu_msg;
ros::Publisher imu_pub("raw_imu_data", &imu_msg);
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
	nh.initNode();
	nh.advertise(imu_pub);
}

 void loop()
 {
	if((millis()-timer)>=20)  // Main loop runs at 50Hz
	{
		    timer=millis();
			gyro.read();
			compass.readAcc();
			compass.readMag();
			imu_msg.angular_velocity[0]=gyro.g.x;
			imu_msg.angular_velocity[1]=gyro.g.y;
			imu_msg.angular_velocity[2]=gyro.g.z;
			imu_msg.linear_acceleration[0]=compass.a.x;
			imu_msg.linear_acceleration[1]=compass.a.y;
			imu_msg.linear_acceleration[2]=compass.a.z;
			imu_msg.magnetic_field[0]=compass.m.x;
			imu_msg.magnetic_field[1]=compass.m.y;
			imu_msg.magnetic_field[2]=compass.m.z;
			imu_pub.publish( &imu_msg );
			nh.spinOnce();	
	}
	nh.spinOnce();
 }
