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
* @brief  This class contains methods that all clases may call upon
*/
#pragma once

#include <Arduino.h>
#include "Wire.h"
#include "Config.hpp"

//using namespace Configuration;

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

      //Compile time check to see if user has set at least on, but not both...
      static_assert((Config::USE_250_DEGS_SECOND || Config::USE_500_DEGS_SECOND), "You must set either USE_250_DEGS_SECOND or USE_500_DEGS_SECOND in Config.hpp");
      static_assert((!Config::USE_250_DEGS_SECOND || !Config::USE_500_DEGS_SECOND), "Only set USE_250_DEGS_SECOND or USE_500_DEGS_SECOND not both in Config.hpp");
};