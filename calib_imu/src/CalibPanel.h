/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     CalibrationPanel.h
 * \license  BSD-3-License
 */

#ifndef CALIB_PANEL_H_
#define CALIB_PANEL_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#ifndef Q_MOC_RUN
# include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

# include <rviz/panel.h>
#endif


class QLineEdit;
class Sensor3DCalibration;
class CalibrationGenerator;
namespace calib_imu
{
class CalibDisplay;
class SubscriberWrapper;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class CalibPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  CalibPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
  virtual void onInitialize();
  
	/**
	 * @brief Callback for received imu data.
	 * Gets called every time new imu data arrives - the data is split in two messages
	 * (one for gyro and accelerometer as well as one for magnetometer readings)
	 *
	 * @param[in] msg_imu Gyro/accelerometer readings
	 * @param[in] msg_mag Magnetometer readings
	 */
	void imuDataArrived(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::MagneticField::ConstPtr& msg_mag);

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic( const QString& topic );

  // Here we declare some internal slots.
protected Q_SLOTS:

  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateTopic();

  // Then we finish up with protected member variables.
protected:

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* read_topic_editor;

  // The current name of the output topic.
  QString read_topic;

  CalibDisplay * display;
  
  boost::shared_ptr<SubscriberWrapper> subscriber_;

	// The ROS node handle.
	ros::NodeHandle nh;
	Sensor3DCalibration* acc_cal; /**< Container used for calibration data for the accelerometer */
	Sensor3DCalibration* ang_cal; /**< Container used for calibration data for the gyro */
	Sensor3DCalibration* mag_cal; /**< Container used for calibration data for the magnetometer */

	/**
	 * @brief Calculates the calibration data for the three sensors of the imu.
	 */
	CalibrationGenerator* cal_gen;


};

} // end namespace calib_imu

#endif // CALIB_PANEL_H_
