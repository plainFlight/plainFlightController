/* 
* Copyright (c) 2025 P.Cook (alias 'plainFlight')
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
* @file   Mpu6050.hpp
* @brief  This class contains methods that handle communications with the MPU6050.
*/
#pragma once

#include <Arduino.h>
#include <cstdint>
#include "ESP32_SoftWire.h"
// #include "CommonTypes.hpp"
#include "Config.hpp"

/**
* @brief    Converts an AircraftDir enum value to a unit vector in {front, left, up} coordinate space.
* @param    d  The aircraft direction to convert.
* @return   A 3-element array representing the unit vector for the given direction.
*/
constexpr std::array<int16_t, 3> dirToVec(AircraftDir d) 
{
    if (d == AircraftDir::FRONT) return {{  1,  0,  0 }};
    if (d == AircraftDir::BACK)  return {{ -1,  0,  0 }};
    if (d == AircraftDir::LEFT)  return {{  0,  1,  0 }};
    if (d == AircraftDir::RIGHT) return {{  0, -1,  0 }};
    if (d == AircraftDir::UP)    return {{  0,  0,  1 }};
    if (d == AircraftDir::DOWN)  return {{  0,  0, -1 }};
    return {{ 0, 0, 0 }};
}

/**
* @brief    Builds a 3x3 rotation matrix at compile time to transform IMU-space
*           acceleration and rotation vectors into aircraft coordinate space.
*           The Z axis is derived as the cross product of the configured IMU X and Y axes.
* @return   A 3x3 matrix where each row maps an aircraft axis to the corresponding IMU axis.
*/
namespace Orientation 
{
  using Matrix3x3 = std::array<std::array<int16_t, 3>, 3>;

  // Build the rotation matrix required to transform the acceleration and rotation
  // vectors back into the aircraft coordinate space
  constexpr Matrix3x3 getMatrix() 
  {
    // x and y are given from the configuration
    std::array<int16_t, 3> imuX = dirToVec(Config::IMU_PLUS_X);
    std::array<int16_t, 3> imuY = dirToVec(Config::IMU_PLUS_Y);

    // Z is orthogonal to x and y, we can derive it using the cross product
    // of IMU X and IMU Y. Note the sign change for the second term.
    std::array<int16_t, 3> imuZ = {
        int16_t(imuX[1]*imuY[2] - imuX[2]*imuY[1]),
        int16_t(imuX[2]*imuY[0] - imuX[0]*imuY[2]),  // negative
        int16_t(imuX[0]*imuY[1] - imuX[1]*imuY[0])
    };

    // --- Now fill all 9 elements ---
    // Row index = aircraft axis {FRONT/BACK=0, LEFT/RIGHT=1, UP/DOWN=2}
    // Column index = IMU axis {X=0, Y=1, Z=2}
    return Matrix3x3{{
        {{imuX[0], imuY[0], imuZ[0]}},  // aircraft X (FRONT/BACK)
        {{imuX[1], imuY[1], imuZ[1]}},  // aircraft Y (LEFT/RIGHT)
        {{imuX[2], imuY[2], imuZ[2]}}   // aircraft Z (UP/DOWN)
    }};
  }

  /**
  * @brief    Precomputed rotation matrix derived from Config::IMU_PLUS_X and Config::IMU_PLUS_Y.
  *           Used at runtime to remap raw IMU data into aircraft coordinate space.
  */
  inline constexpr Matrix3x3 final_matrix = getMatrix();

  // This relies on the AircraftDir enum having sequential opposites to check for sharing opposited direction
  static_assert(
    Config::IMU_PLUS_X != Config::IMU_PLUS_Y &&
    Config::IMU_PLUS_X != (static_cast<AircraftDir>(static_cast<int>(Config::IMU_PLUS_Y) ^ 1)),
    "Invalid IMU Configuration: IMU_PLUS_X and IMU_PLUS_Y cannot share the same physical dimension!"
  );
};

/**
* @brief    Multiplies a single row of the orientation matrix against the raw IMU axis values,
*           remapping one aircraft axis from IMU space. Since the matrix contains only unit vectors
*           (0, 1, or -1), only one column per row will be non-zero.
* @tparam   Row  The aircraft axis row index (0=front/back, 1=left/right, 2=up/down).
* @param    x    Raw IMU X axis value.
* @param    y    Raw IMU Y axis value.
* @param    z    Raw IMU Z axis value.
* @return   The remapped axis value in aircraft coordinate space.
*/
template<size_t Row>
inline int16_t remapAxis(int16_t x, int16_t y, int16_t z) 
{
  if constexpr (Orientation::final_matrix[Row][0] != 0) return x * Orientation::final_matrix[Row][0];
  else if constexpr (Orientation::final_matrix[Row][1] != 0) return y * Orientation::final_matrix[Row][1];
  else return z * Orientation::final_matrix[Row][2];
}

class Mpu6050
{
  public:
    struct MpuData
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

    //MPU6050 register data
    static constexpr uint8_t MPU6050_ADD            = 0x68U;
    //CONFIG register items & settings...
    static constexpr uint8_t EXT_SYNC_SET           = 0x00U;      
    static constexpr uint8_t DLPF_CFG_5HZ           = 0x06U;
    //GYRO_CONFIG register items & settings...
    static constexpr uint8_t FS_SEL_250             = 0x00U;
    static constexpr uint8_t FS_SEL_500             = 0x01U;
    static constexpr uint8_t G_ST                   = 0x00U;  //Gyro self test - not currently used
    //ACCEL_CONFIG register items & settings...
    static constexpr uint8_t AFS_SEL_2G             = 0x00U;
    static constexpr uint8_t AFS_SEL_16G            = 0x03U;
    static constexpr uint8_t A_ST                   = 0x00U;  //Acc self test - not currently used
    //MPU6050 registers
    static constexpr uint8_t CONFIG                 = (EXT_SYNC_SET | DLPF_CFG_5HZ);
    static constexpr uint8_t GYRO_CONFIG_250        = ((G_ST << 5U) | (FS_SEL_250 << 3U));
    static constexpr uint8_t GYRO_CONFIG_500        = ((G_ST << 5U) | (FS_SEL_500 << 3U));
    static constexpr uint8_t ACCEL_CONFIG           = ((A_ST << 5U) | (AFS_SEL_16G << 3U));

    //Non register data
    static constexpr uint32_t I2C_CLK_1MHZ          = 1000000U;
    static constexpr float GYRO_SCALE_FACTOR_250    = 131.0f;
    static constexpr float GYRO_SCALE_FACTOR_500    = 65.5f;
    static constexpr float ACCEL_SCALE_FACTOR_2G    = 16384.0f;
    static constexpr float ACCEL_SCALE_FACTOR_4G    = 8192.0f;
    static constexpr float ACCEL_SCALE_FACTOR_8G    = 4096.0f;
    static constexpr float ACCEL_SCALE_FACTOR_16G   = 2048.0f;
    
    //Methods
    Mpu6050();
    void begin();
    void initialise();
    void reset();
    void setGyroConfig(const uint8_t gyroScale);
    void setAccelerometerConfig(const uint8_t accelScale);
    void setConfig(const uint8_t dlpfHz);
    bool readData(MpuData* const data);
    uint8_t readRegister(const uint8_t theRegister);
    uint8_t whoAmI();

    private:
      float m_scaleFactor;

      //Objects
      SoftWire i2c;

};