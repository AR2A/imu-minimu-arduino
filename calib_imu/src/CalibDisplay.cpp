#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/helpers/color.h>

#include "CalibDisplay.h"

using namespace std;

namespace calib_imu
{
	
	
	
CalibDisplay::CalibDisplay()
{

}

void CalibDisplay::onInitialize()
{
  Display::onInitialize();
  visuals_.reset(new rviz::PointCloud());
  scene_node_->attachObject(visuals_.get());
   visuals_->setRenderMode(rviz::PointCloud::RM_BOXES );
}

CalibDisplay::~CalibDisplay()
{
}

// Clear the visuals by deleting their objects.
void CalibDisplay::reset()
{
  Display::reset();
  visuals_->clear();

}

void CalibDisplay::DrawPoint(double x, double y, double z, rviz::Color color){
   rviz::PointCloud::Point * p =new rviz::PointCloud::Point();
   p->setColor(color.r_, color.g_, color.b_);
   p->position = Ogre::Vector3(x,y,z);
   visuals_->addPoints(p,1);

   //cout << "Point " << x <<" "<<y<<" "<<z<<endl;
}

void CalibDisplay::DrawPlane(arma::vec normal, arma::vec center){
Ogre::Vector3 nm(normal(0),normal(1),normal(2));
Ogre::Vector3 ct(center(0),center(1),center(2));

   cout << "Normal " << normal(0) <<" "<<normal(1)<<" "<<normal(2)<<endl;
   cout << "Center " << center(0)<<" "<<center(1)<<" "<<center(2)<<endl;
}

} // end namespace calib_imu

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(calib_imu::CalibDisplay,rviz::Display )
