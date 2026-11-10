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
* @file   Lsm6dsox.hpp
* @brief  This class contains methods that handle communications with the LSM6DSOX.
* @note   Method set and call shape match Mpu6050's, so Config::SelectedImu can hold
*         either type with no other code changes required.
*/
#pragma once

#include <Arduino.h>
#include <cstdint>
#include "Config.hpp"
#include "CommonTypes.hpp"
#include "Orientation.hpp"
#include "ImuSample.hpp"
#include "SoftI2CBus.hpp"

class Lsm6dsox
{
  public:
    static constexpr uint8_t LSM6DSOX_WHOAMI_VALUE  = 0x6CU;
    static constexpr uint8_t WHOAMI_VALUE           = LSM6DSOX_WHOAMI_VALUE;  //Uniform name, so IMU.cpp can check Config::SelectedImu::WHOAMI_VALUE regardless of the device selected.
    //I2C address - unlike MPU6050's, this isn't fixed in silicon: it's a hardware
    //strapping choice on the breakout board (SDO/SA0 pin tied high = 0x6B, tied low =
    //0x6A). 0x6A is the default here because it's what a no-jumper module resolves to.
    //If your module is strapped differently, edit this constant directly - same as you
    //would for a clone MPU6050 with a non-standard address.
    static constexpr uint8_t LSM6DSOX_I2C_ADDRESS   = 0x6AU;
    //Registers
    static constexpr uint8_t WHO_AM_I               = 0x0FU;
    static constexpr uint8_t CTRL1_XL               = 0x10U;
    static constexpr uint8_t CTRL2_G                = 0x11U;
    static constexpr uint8_t CTRL3_C                = 0x12U;
    static constexpr uint8_t CTRL4_C                = 0x13U;
    static constexpr uint8_t CTRL6_C                = 0x15U;
    static constexpr uint8_t OUT_TEMP_L             = 0x20U;
    //CTRL3_C items & settings...
    static constexpr uint8_t SW_RESET               = 0x01U;  //Self-clearing.
    static constexpr uint8_t IF_INC                 = 0x04U;  //Register address auto-increment, default-on. Needed for burst reads.
    static constexpr uint8_t BDU                    = 0x40U;  //Block Data Update - freeze registers during a burst read.
    static constexpr uint8_t CTRL3_C_BDU_IF_INC     = (BDU | IF_INC);
    //CTRL1_XL / CTRL2_G ODR field (bits [7:4]), high performance mode.
    //Accelerometer and gyro both run at 1667Hz, the closest rate above the 1kHz loop, so every
    //loop reads a fresh sample. All gyro LPF1 bandwidths (FTYPE 000-111) are available at this rate.
    static constexpr uint8_t ODR_1667HZ             = 0x80U;
    //CTRL1_XL FS_XL field (bits [3:2]) - accelerometer full scale.
    static constexpr uint8_t FS_XL_16G              = 0x04U;  //FS_XL[1:0] = 01
    //CTRL1_XL LPF2_XL_EN bit - enables the accelerometer's second-stage digital low-pass filter.
    static constexpr uint8_t LPF2_XL_EN             = 0x02U;  //Bit 1.
    //CTRL2_G FS_G field (bits [3:2]) and FS_125 bit (bit 1) - gyro full scale.
    static constexpr uint8_t FS_125                 = 0x02U;
    static constexpr uint8_t FS_G_250               = 0x00U;  //FS_G[1:0] = 00
    static constexpr uint8_t FS_G_500               = 0x04U;  //FS_G[1:0] = 01
    static constexpr uint8_t FS_G_1000              = 0x08U;  //FS_G[1:0] = 10
    static constexpr uint8_t FS_G_2000              = 0x0CU;  //FS_G[1:0] = 11
    //Combined register values written during initialise()
    static constexpr uint8_t ACCEL_CONFIG           = (ODR_1667HZ | FS_XL_16G | LPF2_XL_EN);
    static constexpr uint8_t GYRO_CONFIG_125        = (ODR_1667HZ | FS_125);
    static constexpr uint8_t GYRO_CONFIG_250        = (ODR_1667HZ | FS_G_250);
    static constexpr uint8_t GYRO_CONFIG_500        = (ODR_1667HZ | FS_G_500);
    static constexpr uint8_t GYRO_CONFIG_1000       = (ODR_1667HZ | FS_G_1000);
    static constexpr uint8_t GYRO_CONFIG_2000       = (ODR_1667HZ | FS_G_2000);
    //CTRL4_C item - gyro LPF1 digital filter enable (only active in high performance mode,
    //which the ODR settings above already select). The MPU6050 has an equivalent filter
    //(its CONFIG register DLPF_CFG=5Hz) hardwired on; the LSM6DSOX has no filtering unless
    //this is explicitly turned on, so raw samples are otherwise close to full-bandwidth.
    static constexpr uint8_t LPF1_SEL_G             = 0x02U;  //Bit 1.
    //CTRL6_C FTYPE[2:0] field (bits [2:0]) - gyro LPF1 bandwidth. The Hz figures below are
    //ST AN5272 Table 15 values at 1667Hz ODR.
    static constexpr uint8_t FTYPE_50HZ             = 0x05U;  //FTYPE[2:0] = 101, 49.8Hz @1667Hz ODR
    static constexpr uint8_t FTYPE_25HZ             = 0x06U;  //FTYPE[2:0] = 110, 25.1Hz @1667Hz ODR
    static constexpr uint8_t FTYPE_12_5HZ           = 0x07U;  //FTYPE[2:0] = 111, 12.5Hz @1667Hz ODR
    //CTRL8_XL register - HPCF_XL[2:0] (bits [7:5]) selects the LPF2 cutoff as a divisor of accel ODR,
    //when HP_SLOPE_XL_EN (bit 2, left at its 0 reset value) selects the low-pass path.
    static constexpr uint8_t CTRL8_XL               = 0x17U;
    static constexpr uint8_t HPCF_XL_ODR_DIV_200    = 0xA0U;  //101 << 5 : ODR/200 -> 8.33Hz @1667Hz ODR
    static constexpr uint8_t HPCF_XL_ODR_DIV_100    = 0x80U;  //100 << 5 : ODR/100 -> 16.7Hz @1667Hz ODR
    static constexpr uint8_t HPCF_XL_ODR_DIV_20     = 0x40U;  //010 << 5 : ODR/20  -> 83.3Hz @1667Hz ODR
    //Accel LPF2 cutoff in use - change this to try a different cutoff.
    static constexpr uint8_t ACCEL_LPF2_HPCF        = HPCF_XL_ODR_DIV_100;
    //Gyro LPF1 bandwidth in use - change this to try a different cutoff.
    static constexpr uint8_t GYRO_LPF1_FTYPE        = FTYPE_25HZ;
    
    //Non register data
    static constexpr uint32_t I2C_CLK_1MHZ          = 1000000U;  //LSM6DSOX supports I2C Fast Mode+ up to 1 MHz.
    static constexpr uint32_t TURN_ON_DELAY_MS      = 35U;       //Datasheet: typical turn-on time from supply stable to valid output.
    static constexpr float GYRO_SCALE_FACTOR_125    = 228.57f;   //1000 / 4.375 mdps/LSB
    static constexpr float GYRO_SCALE_FACTOR_250    = 114.29f;   //1000 / 8.75  mdps/LSB
    static constexpr float GYRO_SCALE_FACTOR_500    = 57.14f;    //1000 / 17.5  mdps/LSB
    static constexpr float GYRO_SCALE_FACTOR_1000   = 28.57f;    //1000 / 35    mdps/LSB
    static constexpr float GYRO_SCALE_FACTOR_2000   = 14.29f;    //1000 / 70    mdps/LSB
    static constexpr float ACCEL_SCALE_FACTOR_16G   = 2049.18f;  //1000 / 0.488 mg/LSB

    //This driver implements all five gyro ranges the shared GyroRate enum offers.
    static_assert(Config::GYRO_RATE == GyroRate::IS_125_DEGS_SECOND  ||
                  Config::GYRO_RATE == GyroRate::IS_250_DEGS_SECOND  ||
                  Config::GYRO_RATE == GyroRate::IS_500_DEGS_SECOND  ||
                  Config::GYRO_RATE == GyroRate::IS_1000_DEGS_SECOND ||
                  Config::GYRO_RATE == GyroRate::IS_2000_DEGS_SECOND,
                  "LSM6DSOX does not support the selected GYRO_RATE.");

    using DeviceBus = SoftI2CBus;

    //Methods
    Lsm6dsox();
    void begin();
    void initialise();
    bool readData(ImuRawData* const data);
    void writeRegister(const uint8_t theRegister, const uint8_t theValue);
    uint8_t readRegister(const uint8_t theRegister);
    uint8_t whoAmI() {  return readRegister(WHO_AM_I);}  //Get who am I data

    private:
      float m_scaleFactor = 0.0f;

      //Objects
      DeviceBus m_bus;

};
