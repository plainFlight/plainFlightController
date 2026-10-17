/*
* Original File Author: D. Gamble (Github: Cyberslug)
*
* Copyright (c) 2026 P.Cook (alias 'plainFlight')
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

/**
* @file   ImuSample.hpp
* @brief  Device-agnostic IMU sample storage and finishing logic, shared by every
*         sensor driver (Mpu6050, Lsm6dsox, ...).
* @note   ImuRawData's field layout is unchanged from the per-driver struct it
*         replaces - only its scope moved, from nested-per-class to shared.
*/
#pragma once

#include <cstdint>
#include <Arduino.h>
#include "Orientation.hpp"
#include "InternalConfig.hpp"

struct ImuRawData
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
  int16_t temperature;
};

/**
* @brief    Remaps one sample's raw accel/gyro axes into aircraft coordinate space,
*           converts them to physical units, and (if enabled) prints debug output.
*           Called by each driver's readData() once its device-specific raw values
*           have been parsed - this is the part of readData() that does not differ
*           between devices.
* @param    rawAccel_X/Y/Z   Raw accelerometer axis values, in IMU coordinate space.
* @param    rawGyro_X/Y/Z    Raw gyro axis values, in IMU coordinate space.
* @param    gyroScaleFactor  Device- and range-specific LSB-per-deg/s divisor.
* @param    accelScaleFactor Device-specific LSB-per-g divisor.
* @param    data             Sample to fill in. data->gyroOffset_X/Y/Z must already
*                             hold the calibrated offsets; this function reads but
*                             does not modify them.
*/
inline void finaliseImuSample(const int16_t rawAccel_X, const int16_t rawAccel_Y, const int16_t rawAccel_Z,
                               const int16_t rawGyro_X, const int16_t rawGyro_Y, const int16_t rawGyro_Z,
                               const float gyroScaleFactor, const float accelScaleFactor,
                               ImuRawData* const data)
{
  // Remap axes using compile-time matrix evaluation
  data->rawAccel_X = remapAxis<0>(rawAccel_X, rawAccel_Y, rawAccel_Z);
  data->rawAccel_Y = remapAxis<1>(rawAccel_X, rawAccel_Y, rawAccel_Z);
  data->rawAccel_Z = remapAxis<2>(rawAccel_X, rawAccel_Y, rawAccel_Z);

  data->rawGyro_X  = remapAxis<0>(rawGyro_X, rawGyro_Y, rawGyro_Z);
  data->rawGyro_Y  = remapAxis<1>(rawGyro_X, rawGyro_Y, rawGyro_Z);
  data->rawGyro_Z  = remapAxis<2>(rawGyro_X, rawGyro_Y, rawGyro_Z);

  // Scaling math
  data->gyro_X  = static_cast<float>(data->rawGyro_X - data->gyroOffset_X) / gyroScaleFactor;
  data->accel_X = static_cast<float>(data->rawAccel_X) / accelScaleFactor;
  data->gyro_Y  = static_cast<float>(data->rawGyro_Y - data->gyroOffset_Y) / gyroScaleFactor;
  data->accel_Y = static_cast<float>(data->rawAccel_Y) / accelScaleFactor;
  data->gyro_Z  = static_cast<float>(data->rawGyro_Z - data->gyroOffset_Z) / gyroScaleFactor;
  data->accel_Z = static_cast<float>(data->rawAccel_Z) / accelScaleFactor;

  if constexpr(InternalConfig::DEBUG_IMU)
  {
    Serial.print("\t gx:");
    Serial.print(data->gyro_X);
    Serial.print("\t gy:");
    Serial.print(data->gyro_Y);
    Serial.print("\t gz:");
    Serial.print(data->gyro_Z);
    Serial.print("\t ax:");
    Serial.print(data->accel_X);
    Serial.print("\t ay:");
    Serial.print(data->accel_Y);
    Serial.print("\t az:");
    Serial.println(data->accel_Z);
  }
}
