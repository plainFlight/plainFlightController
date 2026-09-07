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
* @file   Lsm6dsox.cpp
* @brief  This class contains methods that handle communications with the LSM6DSOX.
*/

#include "Lsm6dsox.hpp"
#include "InternalConfig.hpp"
#include "CommonTypes.hpp"

/**
* @brief    Constructor that sets the desired gyro rate.
*/
Lsm6dsox::Lsm6dsox()
{
  if constexpr(Config::GYRO_RATE == GyroRate::IS_125_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_125;
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_250_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_250;
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_500_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_500;
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_1000_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_1000;
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_2000_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_2000;
  }
}


/**
* @brief    Initialises the LSM6DSOX.
* @note     Register writes only.
*/
void
Lsm6dsox::initialise()
{
  begin();
  writeRegister(CTRL3_C, SW_RESET); // Reset
  delay(TURN_ON_DELAY_MS);

  //BDU=1 so a burst read always returns one coherent sample (registers freeze after
  //the first byte is read, release after the last); IF_INC=1 (default) preserved so
  //readData()'s sequential burst read keeps working.
  writeRegister(CTRL3_C, CTRL3_C_BDU_IF_INC); // Set configuration (BDU / IF_INC)

  if constexpr(Config::GYRO_RATE == GyroRate::IS_125_DEGS_SECOND)
  {
    writeRegister(CTRL2_G, GYRO_CONFIG_125); // Set gyro configuration
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_250_DEGS_SECOND)
  {
    writeRegister(CTRL2_G, GYRO_CONFIG_250); // Set gyro configuration
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_500_DEGS_SECOND)
  {
    writeRegister(CTRL2_G, GYRO_CONFIG_500); // Set gyro configuration
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_1000_DEGS_SECOND)
  {
    writeRegister(CTRL2_G, GYRO_CONFIG_1000); // Set gyro configuration
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_2000_DEGS_SECOND)
  {
    writeRegister(CTRL2_G, GYRO_CONFIG_2000); // Set gyro configuration
  }

  //Gyro LPF1 digital filter. The LSM6DSOX has no equivalent of the MPU6050's fixed 5Hz
  //DLPF unless this is explicitly enabled, so without it raw samples are close to
  //full-bandwidth, see GYRO_LPF1_FTYPE in Lsm6dsox.hpp to change the cutoff.
  writeRegister(CTRL4_C, LPF1_SEL_G); // Set gyro low pass filter
  writeRegister(CTRL6_C, GYRO_LPF1_FTYPE); // Set gyro filter bandwidth

  writeRegister(CTRL8_XL, ACCEL_LPF2_HPCF);  // Set accel LPF2 filter bandwidth
  writeRegister(CTRL1_XL, ACCEL_CONFIG); // Set accelerometer configuration
}


/**
* @brief    Sets up and start the SoftWire I2C transfer.
*/
void
Lsm6dsox::begin()
{
  i2c.begin(Config::ESP32S3.I2C_SDA,Config::ESP32S3.I2C_SCL,I2C_CLK_1MHZ);
  i2c.begin();
}


/**
* @brief    Reads the temperature, gyro and accelerometer data from the LSM6DSOX.
* @param    Pointer to data structure where imu data is stored.
* @return   true when data successfully read.
* @note     Burst-read order is temp, gyro, accel. The opposite of the MPU6050's
*           accel, temp, gyro order and each 16-bit value is little-endian (L
*           byte then H byte), the opposite of the MPU6050's big-endian layout.
*/
bool
Lsm6dsox::readData(ImuRawData* const data)
{
  i2c.beginTransmission(InternalConfig::LSM6DSOX_I2C_ADDRESS);
  i2c.write(OUT_TEMP_L);               //Register
  i2c.endTransmission(false);
  const uint8_t bytesReceived = i2c.requestFrom(InternalConfig::LSM6DSOX_I2C_ADDRESS, 14, true);  //Get temp, gyro and accelerometer data

  if (14U == bytesReceived)
  {
    //Little-endian: low byte first, then high byte.
    const uint8_t tempL = static_cast<uint8_t>(i2c.read());
    const uint8_t tempH = static_cast<uint8_t>(i2c.read());
    data->temperature   = (static_cast<int16_t>(tempH) << 8) | static_cast<int16_t>(tempL);

    const uint8_t gXL = static_cast<uint8_t>(i2c.read());
    const uint8_t gXH = static_cast<uint8_t>(i2c.read());
    const uint8_t gYL = static_cast<uint8_t>(i2c.read());
    const uint8_t gYH = static_cast<uint8_t>(i2c.read());
    const uint8_t gZL = static_cast<uint8_t>(i2c.read());
    const uint8_t gZH = static_cast<uint8_t>(i2c.read());
    const int16_t rawG_X = (static_cast<int16_t>(gXH) << 8) | static_cast<int16_t>(gXL);
    const int16_t rawG_Y = (static_cast<int16_t>(gYH) << 8) | static_cast<int16_t>(gYL);
    const int16_t rawG_Z = (static_cast<int16_t>(gZH) << 8) | static_cast<int16_t>(gZL);

    const uint8_t aXL = static_cast<uint8_t>(i2c.read());
    const uint8_t aXH = static_cast<uint8_t>(i2c.read());
    const uint8_t aYL = static_cast<uint8_t>(i2c.read());
    const uint8_t aYH = static_cast<uint8_t>(i2c.read());
    const uint8_t aZL = static_cast<uint8_t>(i2c.read());
    const uint8_t aZH = static_cast<uint8_t>(i2c.read());
    const int16_t rawA_X = (static_cast<int16_t>(aXH) << 8) | static_cast<int16_t>(aXL);
    const int16_t rawA_Y = (static_cast<int16_t>(aYH) << 8) | static_cast<int16_t>(aYL);
    const int16_t rawA_Z = (static_cast<int16_t>(aZH) << 8) | static_cast<int16_t>(aZL);

    finaliseImuSample(rawA_X, rawA_Y, rawA_Z, rawG_X, rawG_Y, rawG_Z,
                       m_scaleFactor, ACCEL_SCALE_FACTOR_16G, data);

    return true;
  }
  else
  {
    if constexpr(InternalConfig::DEBUG_IMU)
    {
      Serial.println("LSM6DSOX read error  !");
    }

    return false;
  }
}


/**
* @brief    Writes data to a register.
* @param    theRegister representing the desired register address to write.
* @param    theValue the value to write.
*/
void 
Lsm6dsox::writeRegister(const uint8_t theRegister, const uint8_t theValue)
{
  i2c.beginTransmission(InternalConfig::LSM6DSOX_I2C_ADDRESS);
  i2c.write(theRegister);     //Register
  i2c.write(theValue);        //Data
  i2c.endTransmission(true);
}


/**
* @brief    Reads a register data value.
* @param    Data representing the desired register address to read.
*/
uint8_t
Lsm6dsox::readRegister(const uint8_t theRegister)
{
  i2c.beginTransmission(InternalConfig::LSM6DSOX_I2C_ADDRESS);
  i2c.write(theRegister);   //Register
  i2c.endTransmission(false);
  i2c.requestFrom(InternalConfig::LSM6DSOX_I2C_ADDRESS, 1, true);
  return static_cast<uint8_t>(i2c.read());
}

