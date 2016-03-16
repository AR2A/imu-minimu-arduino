#ifndef _ROS_hector_elevation_msgs_ElevationMapMetaData_h
#define _ROS_hector_elevation_msgs_ElevationMapMetaData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"

namespace hector_elevation_msgs
{

  class ElevationMapMetaData : public ros::Msg
  {
    public:
      ros::Time map_load_time;
      float resolution_xy;
      float resolution_z;
      float min_elevation;
      float max_elevation;
      int16_t zero_elevation;
      uint32_t width;
      uint32_t height;
      geometry_msgs::Pose origin;

    ElevationMapMetaData():
      map_load_time(),
      resolution_xy(0),
      resolution_z(0),
      min_elevation(0),
      max_elevation(0),
      zero_elevation(0),
      width(0),
      height(0),
      origin()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->map_load_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->map_load_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->map_load_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->map_load_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_load_time.sec);
      *(outbuffer + offset + 0) = (this->map_load_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->map_load_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->map_load_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->map_load_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_load_time.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->resolution_xy);
      offset += serializeAvrFloat64(outbuffer + offset, this->resolution_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_elevation);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_elevation);
      union {
        int16_t real;
        uint16_t base;
      } u_zero_elevation;
      u_zero_elevation.real = this->zero_elevation;
      *(outbuffer + offset + 0) = (u_zero_elevation.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zero_elevation.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->zero_elevation);
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      *(outbuffer + offset + 0) = (this->height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      offset += this->origin.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->map_load_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->map_load_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->map_load_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->map_load_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->map_load_time.sec);
      this->map_load_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->map_load_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->map_load_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->map_load_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->map_load_time.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->resolution_xy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->resolution_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_elevation));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_elevation));
      union {
        int16_t real;
        uint16_t base;
      } u_zero_elevation;
      u_zero_elevation.base = 0;
      u_zero_elevation.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zero_elevation.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->zero_elevation = u_zero_elevation.real;
      offset += sizeof(this->zero_elevation);
      this->width =  ((uint32_t) (*(inbuffer + offset)));
      this->width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->width);
      this->height =  ((uint32_t) (*(inbuffer + offset)));
      this->height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->height);
      offset += this->origin.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "hector_elevation_msgs/ElevationMapMetaData"; };
    const char * getMD5(){ return "6c887873faf3a1d55d884bdcc92b9241"; };

  };

}
#endif