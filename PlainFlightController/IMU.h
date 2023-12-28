/* 
* Copyright (c) 2023,2024 P.Cook (alias 'plainFlight')
*
* This file is part of the PlainFlightController distribution (https://github.com/plainFlight/plainFlightController).
* 
* This program is free software: you can redistribute it and/or modify  
* it under the terms of the GNU General Public License as published by  
* the Free Software Foundation, version 3.
*
* This program is distributed in the hope that it will be useful, but 
* WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IMU_h
#define IMU_h

#include "PIDF.h"

//Gyro defines
#ifdef GYRO_FS_SEL_250
#define GYRO_SCALE 0
#define GYRO_SCALE_FACTOR 131.0
#else
#define GYRO_SCALE 1
#define GYRO_SCALE_FACTOR 65.5
#endif

#define DLPF_5HZ 6


//Accelerometer defines
#define ACCEL_FS_SEL_2 0
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0

//Gyro calibration defines
#define CALIBRATE_MAX_MOTION 1000
#define CALIBRATE_COUNTS 1000

//PID wind up limits
#define I_WIND_UP_LIMIT     1000000
#define I_WIND_UP_LIMIT_YAW 1000000
#define D_WIND_UP_LIMIT     500000

typedef struct
{
  float accel_X;
  float accel_Y;
  float accel_Z;
  float gyro_X;
  float gyro_Y;
  float gyro_Z;
  int16_t rawGyro_X;
  int16_t rawGyro_Y;
  int16_t rawGyro_Z;
  int16_t rawAccel_X;
  int16_t rawAccel_Y;
  int16_t rawAccel_Z;
  int16_t gyroOffset_X;
  int16_t gyroOffset_Y;
  int16_t gyroOffset_Z;
  float roll;
  float pitch;
  float Yaw;
  bool calibrated;
} IMU_Data;

typedef struct
{
  Gains pitch;
  Gains roll;
  Gains yaw;
} Axis_Gains;

//Instantiate PIDF's
PIDF rollPIF(I_WIND_UP_LIMIT_YAW, D_WIND_UP_LIMIT);
PIDF pitchPIF(I_WIND_UP_LIMIT, D_WIND_UP_LIMIT);
PIDF yawPIF(I_WIND_UP_LIMIT_YAW, D_WIND_UP_LIMIT); 
//Global variables
IMU_Data imu = {0};
float timeDelta = 0.0f;  //Loop time delta calculated by loopRateControl & used by madgewick filter


Axis_Gains gains[2] = {
  //Rate mode gains
  { { RATE_PITCH_P,     RATE_PITCH_I,     RATE_PITCH_D,     RATE_PITCH_F },
    { RATE_ROLL_P,      RATE_ROLL_I,      RATE_ROLL_D,      RATE_ROLL_F },
    { RATE_YAW_P,       RATE_YAW_I,       RATE_YAW_D,       RATE_YAW_F } },     //Yaw, do not recommend i gain for conventional plane that has ailerons as it will fight the aileron turn
  //Levelled mode gains - FF gives Tx stick more strength but will cause small overshoot of set max angle, use i gain with caution.
  { { LEVELLED_PITCH_P, LEVELLED_PITCH_I, LEVELLED_PITCH_D, LEVELLED_PITCH_F }, //Pitch
    { LEVELLED_ROLL_P,  LEVELLED_ROLL_I,  LEVELLED_ROLL_D,  LEVELLED_ROLL_F },  //Roll
    { HEADING_HOLD_P,   HEADING_HOLD_I,   HEADING_HOLD_D,   HEADING_HOLD_F } }  //We use this is for heading hold gains
};


#ifdef REVERSE_ROLL_CORRECTIONS
  #define GYRO_X (int32_t)(-imu.gyro_X * 100.0f)
#else
  #define GYRO_X (int32_t)(imu.gyro_X * 100.0f)
#endif

#ifdef REVERSE_PITCH_CORRECTIONS
  #define GYRO_Y (int32_t)(-imu.gyro_Y * 100.0f)
#else
  #define GYRO_Y (int32_t)(imu.gyro_Y * 100.0f)
#endif

#ifdef REVERSE_YAW_CORRECTIONS
  #define GYRO_Z (int32_t)(-imu.gyro_Z * 100.0f)
#else
  #define GYRO_Z (int32_t)(imu.gyro_Z * 100.0f)
#endif

#endif