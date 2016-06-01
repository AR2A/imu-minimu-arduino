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

/**************************************************************************************
 * CLASSES
 **************************************************************************************/

namespace Ogre {
class SceneNode;
class SceneManager;
}

namespace rviz {

class PointCloud;
class Color;
}

namespace calib_imu {

class CalibDisplay: public rviz::Display {
    Q_OBJECT
  public:
    CalibDisplay();
    virtual ~CalibDisplay();

    void DrawPoint(double x, double y, double z, rviz::Color color);
    void DrawPlane(arma::vec normal, arma::vec center);
    void Clear();

  protected:
    virtual void onInitialize();
    virtual void reset();

  private:
    boost::shared_ptr<rviz::PointCloud> visuals_;

};

} // end namespace calib_imu

#endif // CALIB_DISPLAY_H_
