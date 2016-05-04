/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     CalibrationGenerator.cpp
 * \license  BSD-3-License
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <string>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>

#include <rviz/helpers/color.h>
#include <rviz/visualization_manager.h>


#include <armadillo>

#include "CalibrationGenerator.h"
#include "../../process_imu_data/src/Sensor3DCalibration.h"
#include "CalibPanel.h"
#include "CalibDisplay.h"

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


using namespace std;
using namespace arma;

namespace calib_imu
{
	
	class SubscriberWrapper {
		public:
		SubscriberWrapper(ros::NodeHandle & nh){

			//Create the subscribers for both imu porovided messages
			sub_imu_data =
				new message_filters::Subscriber<sensor_msgs::Imu>(nh,ros::names::resolve("imu") + "/data_raw", QUEUE_LENGTH);
				
				sub_mag_data =
			new message_filters::Subscriber<sensor_msgs::MagneticField>(nh,ros::names::resolve("imu") + "/magnetic_field", QUEUE_LENGTH);
			
			//Synchronize both messages to the same timebase (they should be sent by the imu at the 'same' time.)
			sync = new Synchronizer(SyncPolicy(QUEUE_LENGTH),*sub_imu_data,*sub_mag_data);
		}
		
		Synchronizer * GetSynchronizer(){
			return sync;
		}
		
		private:

		
		message_filters::Subscriber<sensor_msgs::Imu> * sub_imu_data;
		message_filters::Subscriber<sensor_msgs::MagneticField> * sub_mag_data;
		Synchronizer * sync;

				
	};
	
	
	CalibPanel::CalibPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Read Topic:" ));
  read_topic_editor = new QLineEdit;
  topic_layout->addWidget( read_topic_editor );



  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

  // Next we make signal/slot connections.
  connect( read_topic_editor, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  
  display=0;
  
	acc_cal=new Sensor3DCalibration(PATH_TO_CALIBRATION, "acc"); 
	ang_cal=new Sensor3DCalibration(PATH_TO_CALIBRATION, "ang");
	mag_cal=new Sensor3DCalibration(PATH_TO_CALIBRATION, "mag");
	
	cal_gen=new CalibrationGenerator(
	/*   Magnetometer zero position amplitude: */ 1.00, /* normalized */
	/* Accelerometer zero position  amplitude: */ 9.81, /* m/s^2 */
	/*          Gyro zero position amplitude: */ 1.00);/* normalized */
	
}


void CalibPanel::imuDataArrived(const sensor_msgs::Imu::ConstPtr& msg_imu, const sensor_msgs::MagneticField::ConstPtr& msg_mag){
	arma::vec acc_vec(3,arma::fill::zeros); /**< Vector representation of one accelerometer reading */
	arma::vec ang_vec(3,arma::fill::zeros); /**< Vector representation of one gyro reading */
	arma::vec mag_vec(3,arma::fill::zeros); /**< Vector representation of one magnetometer reading */
	
	
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

	if(display!=0){
		display->DrawPoint(msg_mag->magnetic_field.x,msg_mag->magnetic_field.y,msg_mag->magnetic_field.z,rviz::Color(1.0,0.0,0.0));
	}
	
    //Process the current dataset (calculate one step of the calibration)
    cal_gen->CalibrationStep(mag_vec,acc_vec,ang_vec);

    //When the calibration is finished store the calibration data to the working directory
    //and finish. (Storing of the data is done by the Sensor3DCalibration objects).
    if(cal_gen->isFinished()) {
        cal_gen->InitialiseCalibrationObjectMag(*mag_cal);
        cal_gen->InitialiseCalibrationObjectAcc(*acc_cal);
        cal_gen->InitialiseCalibrationObjectGyr(*ang_cal);
        cout << "Calibration finished!" << endl;
    }
}

void CalibPanel::updateTopic()
{
  setTopic( read_topic_editor->text() );
}
	
void CalibPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != read_topic )
  {
    read_topic = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( read_topic == "" )
    {
      //
    }
    else
    {
	  //
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

}

void CalibPanel::onInitialize(){
	//Create a display for displaying the magnetometer values.
	display=(CalibDisplay*)vis_manager_->createDisplay("calib_imu/calib_imu_visualization","CalibDisplay",true);
	display->DrawPoint(5,5,5,rviz::Color(1.0,0.0,0.0));
	//Initialize ros node
	nh.setCallbackQueue(vis_manager_->getThreadedQueue ());
	subscriber_.reset(new SubscriberWrapper(nh));
	subscriber_->GetSynchronizer()->registerCallback(&CalibPanel::imuDataArrived,(CalibPanel*)this);
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void CalibPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", read_topic );
}

// Load all configuration data for this panel from the given Config object.
void CalibPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    read_topic_editor->setText( topic );
    updateTopic();
  }
}
	
} // end namespace calib_imu

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(calib_imu::CalibPanel,rviz::Panel )
