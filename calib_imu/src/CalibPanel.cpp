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

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <armadillo>

#include "CalibrationGenerator.h"
#include "../../process_imu_data/src/Sensor3DCalibration.h"

using namespace std;
using namespace arma;

namespace calib_imu
{
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

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", read_topic );
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load( const rviz::Config& config )
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
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::TeleopPanel,rviz::Panel )