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

/**************************************************************************************
 * CLASSES
 **************************************************************************************/

class QLineEdit;
class QPushButton;
class Sensor3DCalibration;
class CalibrationGenerator;
namespace calib_imu {
class CalibDisplay;
class SubscriberWrapper;
class CalibPanelPrivate;


class CalibPanel: public rviz::Panel {

    Q_OBJECT
  public:

    CalibPanel( QWidget* parent = 0 );
    ~CalibPanel();

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

  public Q_SLOTS:
    void setTopic( const QString& topic );

  protected Q_SLOTS:
    void updateTopic();
    void proceedClicked(bool checked);

  protected:

    QLineEdit* read_topic_editor;
    QPushButton* btn_proceed;
    QString read_topic;
    CalibDisplay * display;
	
	
    boost::shared_ptr<SubscriberWrapper> subscriber_;
    ros::NodeHandle nh;
    Sensor3DCalibration* acc_cal; /**< Container used for calibration data for the accelerometer */
    Sensor3DCalibration* ang_cal; /**< Container used for calibration data for the gyro */
    Sensor3DCalibration* mag_cal; /**< Container used for calibration data for the magnetometer */

    CalibrationGenerator* cal_gen;

	/** Private data*/
    CalibPanelPrivate * d;


};

} // end namespace calib_imu

#endif // CALIB_PANEL_H_
