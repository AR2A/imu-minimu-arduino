/**
 * \author   CB
 * \brief    Main file for calibrating the Pololu MinIMU-9.
 * \file     CalibrationPanel.h
 * \license  BSD-3-License
 */

#ifndef CALIB_DISPLAY_H_
#define CALIB_DISPLAY_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.h>

#include <armadillo>
#endif

namespace Ogre
{
class SceneNode;
class SceneManager;
}

namespace rviz
{

class PointCloud;
class Color;
}

namespace calib_imu
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class CalibDisplay: public rviz::Display
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
  CalibDisplay();
  virtual ~CalibDisplay();
  
  void DrawPoint(double x, double y, double z, rviz::Color color);

  void DrawPlane(arma::vec normal, arma::vec center); 
void Clear();

protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:


  // Function to handle an incoming ROS message.
private:
  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::shared_ptr<rviz::PointCloud> visuals_;

};

} // end namespace calib_imu

#endif // CALIB_DISPLAY_H_
