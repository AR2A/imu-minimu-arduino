#ifndef _ROS_avr_imu_Imu_h
#define _ROS_avr_imu_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace avr_imu
{

  class Imu : public ros::Msg
  {
    public:
      int16_t angular_velocity[3];
      int16_t magnetic_field[3];
      int16_t linear_acceleration[3];

    Imu():
      angular_velocity(),
      magnetic_field(),
      linear_acceleration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_angular_velocityi;
      u_angular_velocityi.real = this->angular_velocity[i];
      *(outbuffer + offset + 0) = (u_angular_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocityi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angular_velocity[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_magnetic_fieldi;
      u_magnetic_fieldi.real = this->magnetic_field[i];
      *(outbuffer + offset + 0) = (u_magnetic_fieldi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_magnetic_fieldi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->magnetic_field[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_linear_accelerationi;
      u_linear_accelerationi.real = this->linear_acceleration[i];
      *(outbuffer + offset + 0) = (u_linear_accelerationi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_accelerationi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->linear_acceleration[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_angular_velocityi;
      u_angular_velocityi.base = 0;
      u_angular_velocityi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocityi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angular_velocity[i] = u_angular_velocityi.real;
      offset += sizeof(this->angular_velocity[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_magnetic_fieldi;
      u_magnetic_fieldi.base = 0;
      u_magnetic_fieldi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_magnetic_fieldi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->magnetic_field[i] = u_magnetic_fieldi.real;
      offset += sizeof(this->magnetic_field[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_linear_accelerationi;
      u_linear_accelerationi.base = 0;
      u_linear_accelerationi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_accelerationi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->linear_acceleration[i] = u_linear_accelerationi.real;
      offset += sizeof(this->linear_acceleration[i]);
      }
     return offset;
    }

    const char * getType(){ return "avr_imu/Imu"; };
    const char * getMD5(){ return "8c19c0160486dcd03d80d77430c90626"; };

  };

}
#endif