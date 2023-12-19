#ifndef RADIO_CTRL_H
#define RADIO_CTRL_H

#include "Flight_Ctrl.h"
#include "defines.h"

#define THROTTLE_LOW_THRESHOLD    100    //Sbus data is 11 bit (2048)

typedef enum
{  
  switch_low = 0U,
  switch_middle = 1U,
  switch_high = 2U  
}Switch_States;

typedef struct
{
  int32_t throttle;
  int32_t roll;
  int32_t pitch;
  int32_t yaw;
  bool armSwitch;
  Switch_States modeSwitch;
  Switch_States aux1Switch;
  Switch_States aux2Switch;
  //Note: Ignoring the channels 7-15 from SBUS
  bool failsafe;
  bool throttleIsLow;
  bool headingHold;
  bool newSbusPacket;
}Rx_Commands;

typedef struct
{
  const int32_t servo1;
  const int32_t servo2;
  const int32_t servo3;
  const int32_t servo4;
  const float accRoll;
  const float accPitch;
}Trims;

typedef struct
{
  const uint32_t roll;
  const uint32_t pitch;
  const uint32_t yaw;
}Deadband;

enum
{
  throttle = 0,
  roll,
  pitch,
  yaw,
  aux1,
  aux2,
  aux3,
  aux4
}radio_channel_map;

void processDemands(states currentState);

Rx_Commands rxCommand = {0};

//Use servo trims to set horns in centred position then set control surface mechanically to is neutral posotion.
Trims trim = 
{
  TRIM_SERVO1,          //Roll servo 
  TRIM_SERVO2,          //Roll servo 
  TRIM_SERVO3,          //Pitch servo 
  TRIM_SERVO4,          //Yaw servo 
  TRIM_LEVELLED_ROLL,   //Roll levelled mode adjustment (degrees)
  TRIM_LEVELLED_PITCH   //Pitch levelled mode adjustment (degrees)
}; 

Deadband deadband = {TX_DEABAND_ROLL, TX_DEABAND_PITCH, TX_DEABAND_YAW};

#endif