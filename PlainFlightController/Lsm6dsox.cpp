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
  //readData()'s sequential burst read keeps working on either bus.
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
* @brief    Sets up and starts the I2C bus.
*/
void
Lsm6dsox::begin()
{
  m_bus.begin(Config::ESP32S3.I2C_SDA, Config::ESP32S3.I2C_SCL, I2C_CLK_1MHZ, LSM6DSOX_I2C_ADDRESS);
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
  uint8_t buffer[14];
  const uint8_t bytesReceived = m_bus.readRegisters(OUT_TEMP_L, buffer, 14U);  //Get temp, gyro and accelerometer data

  if (14U == bytesReceived)
  {
    //Little-endian: low byte first, then high byte.
    data->temperature = (static_cast<int16_t>(buffer[1]) << 8) | static_cast<int16_t>(buffer[0]);

    const int16_t rawG_X = (static_cast<int16_t>(buffer[3])  << 8) | static_cast<int16_t>(buffer[2]);
    const int16_t rawG_Y = (static_cast<int16_t>(buffer[5])  << 8) | static_cast<int16_t>(buffer[4]);
    const int16_t rawG_Z = (static_cast<int16_t>(buffer[7])  << 8) | static_cast<int16_t>(buffer[6]);

    const int16_t rawA_X = (static_cast<int16_t>(buffer[9])  << 8) | static_cast<int16_t>(buffer[8]);
    const int16_t rawA_Y = (static_cast<int16_t>(buffer[11]) << 8) | static_cast<int16_t>(buffer[10]);
    const int16_t rawA_Z = (static_cast<int16_t>(buffer[13]) << 8) | static_cast<int16_t>(buffer[12]);

    finaliseImuSample(rawA_X, rawA_Y, rawA_Z, rawG_X, rawG_Y, rawG_Z,
                       m_scaleFactor, ACCEL_SCALE_FACTOR_16G, data);

    return true;
  }
  else
  {
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
  m_bus.writeRegister(theRegister, theValue);
}


/**
* @brief    Reads a register data value.
* @param    Data representing the desired register address to read.
*/
uint8_t
Lsm6dsox::readRegister(const uint8_t theRegister)
{
  return m_bus.readRegister(theRegister);
}
