// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

#include <AP_HAL_Boards.h>

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0
#define LED_DIM                0x11
// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

//Direcciones de memoria para los paquetes

#define  LOG_POSE_MSG          0x01
#define LOG_ERR_MSG            0x0C
#define LOG_CONTROL_MSG        0x05



  enum kpd{
      kp_roll = 85,//80, //60// Cuadri f450 fibra de carbono
      kd_roll = 75,//40
      kp_pitch= 92,//80,//60
      kd_pitch= 82,//40
      kp_yaw  = 100,//100
      kd_yaw  = 70,//70
      kp_z    = 100,//150
      kd_z    = 60,//50
      kp_x    = 34,
      kd_x    = 40,
      kp_y    = 34,
      kd_y    = 40,
  };


#endif // _DEFINES_H
