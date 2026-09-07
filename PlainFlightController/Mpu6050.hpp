/*
* Copyright (c) 2025, 2026 P.Cook (alias 'plainFlight')
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
#include "Config.hpp"
#include "CommonTypes.hpp"
#include "Orientation.hpp"
#include "ImuSample.hpp"

class Mpu6050
{
  public:
    static constexpr uint8_t MPU6050_I2C_ADDRESS    = 0x68U;  //Fixed I2C bus address.
    static constexpr uint8_t MPU6050_WHOAMI_VALUE   = 0x68U;  //Expected WHO_AM_I register (0x75) value. Coincides numerically with the I2C address above, but is a distinct register/value.
    static constexpr uint8_t WHOAMI_VALUE           = MPU6050_WHOAMI_VALUE;  //Uniform name, so IMU.cpp can check Config::SelectedImu::WHOAMI_VALUE regardless of the device selected.

    //Registers
    static constexpr uint8_t PWR_MGMT_1             = 0x6BU;  //Power management 1 - device reset (bit7), sleep (bit6), clock select.
    static constexpr uint8_t CONFIG                 = 0x1AU;  //DLPF configuration register.
    static constexpr uint8_t GYRO_CONFIG            = 0x1BU;  //Gyro full-scale select register.
    static constexpr uint8_t ACCEL_CONFIG           = 0x1CU;  //Accelerometer full-scale select register.
    static constexpr uint8_t ACCEL_XOUT_H           = 0x3BU;  //Burst-read start register (accel, temp, gyro - see readData()).
    static constexpr uint8_t WHO_AM_I               = 0x75U;  //Device ID register.
    //PWR_MGMT_1 items & settings...
    //Clears SLEEP, selects the PLL with Gyro_X oscillator (CLKSEL=1). As recommended by Invensense for clock stability. This value
    //wakes the device from its power-on sleep state, it does not reset all
    //registers to zero.
    static constexpr uint8_t WAKE_PLL_GYRO_X_CLK    = 0x01U;
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
    //Combined register values written during initialise()
    static constexpr uint8_t DLPF_CONFIG_VALUE      = (EXT_SYNC_SET | DLPF_CFG_5HZ);
    static constexpr uint8_t GYRO_CONFIG_250        = ((G_ST << 5U) | (FS_SEL_250 << 3U));
    static constexpr uint8_t GYRO_CONFIG_500        = ((G_ST << 5U) | (FS_SEL_500 << 3U));
    static constexpr uint8_t ACCEL_CONFIG_VALUE     = ((A_ST << 5U) | (AFS_SEL_16G << 3U));

    //Non register data
    static constexpr uint32_t I2C_CLK_1MHZ          = 1000000U;
    static constexpr uint32_t STARTUP_DELAY_MS      = 50U;  //Settling time after waking the device, before configuring it.
    static constexpr float GYRO_SCALE_FACTOR_250    = 131.0f;
    static constexpr float GYRO_SCALE_FACTOR_500    = 65.5f;
    static constexpr float ACCEL_SCALE_FACTOR_2G    = 16384.0f;
    static constexpr float ACCEL_SCALE_FACTOR_4G    = 8192.0f;
    static constexpr float ACCEL_SCALE_FACTOR_8G    = 4096.0f;
    static constexpr float ACCEL_SCALE_FACTOR_16G   = 2048.0f;

    //This driver only implements the 250/500 dps ranges - gate any other selection at compile time.
    static_assert(Config::GYRO_RATE == GyroRate::IS_250_DEGS_SECOND ||
                  Config::GYRO_RATE == GyroRate::IS_500_DEGS_SECOND,
                  "MPU6050 does not support the selected GYRO_RATE - it only offers 250/500 dps.");

    //Methods
    Mpu6050();
    void begin();
    void initialise();
    bool readData(ImuRawData* const data);
    void writeRegister(const uint8_t theRegister, const uint8_t theValue);
    uint8_t readRegister(const uint8_t theRegister);
    uint8_t whoAmI() {  return readRegister(WHO_AM_I);}  //Get who am I data

    private:
      float m_scaleFactor = 0.0f;

      //Objects
      SoftWire i2c;

};
