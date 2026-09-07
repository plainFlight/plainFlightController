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

#include "Mpu6050.hpp"
#include "InternalConfig.hpp"
#include "CommonTypes.hpp"

/**
* @brief    Constructor that sets the desired gyro rate.
*/
Mpu6050::Mpu6050()
{
  if constexpr(Config::GYRO_RATE == GyroRate::IS_250_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_250;
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_500_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_500;
  }
}


/**
* @brief    Initialises the MPU6050.
* @note     Register writes only.
*/
void
Mpu6050::initialise()
{
  begin();
  writeRegister(PWR_MGMT_1, WAKE_PLL_GYRO_X_CLK); // Wake from sleep, select PLL and Gyro X reference clock
  delay(STARTUP_DELAY_MS);

  writeRegister(CONFIG, DLPF_CONFIG_VALUE); // Set DLPF configuration

  if constexpr(Config::GYRO_RATE == GyroRate::IS_250_DEGS_SECOND)
  {
    writeRegister(GYRO_CONFIG, GYRO_CONFIG_250); // Set gyro configuration
  }

  if constexpr(Config::GYRO_RATE == GyroRate::IS_500_DEGS_SECOND)
  {
    writeRegister(GYRO_CONFIG, GYRO_CONFIG_500); // Set gyro configuration
  }

  writeRegister(ACCEL_CONFIG, ACCEL_CONFIG_VALUE); // Set accelerometer configuration
}


/**
* @brief    Sets up and start the SoftWire I2C transfer.
*/
void
Mpu6050::begin()
{
  i2c.begin(Config::ESP32S3.I2C_SDA,Config::ESP32S3.I2C_SCL,I2C_CLK_1MHZ);
  i2c.begin();
}


/**
* @brief    Writes data to a register.
* @param    theRegister representing the desired register address to write.
* @param    theValue the value to write.
*/
void
Mpu6050::writeRegister(const uint8_t theRegister, const uint8_t theValue)
{
  i2c.beginTransmission(MPU6050_I2C_ADDRESS);
  i2c.write(theRegister);   //Register
  i2c.write(theValue);      //Data
  i2c.endTransmission(true);
}


/**
* @brief    Reads the gyro, temperature and accelerometer data form the mpu6050.
* @param    Pointer to data structure where mpu data is stored.
* @return   true when data successfully read.
*/
bool
Mpu6050::readData(ImuRawData* const data)
{
  i2c.beginTransmission(MPU6050_I2C_ADDRESS);
  i2c.write(ACCEL_XOUT_H);               //Register
  i2c.endTransmission(false);
  const uint8_t bytesReceived = i2c.requestFrom(MPU6050_I2C_ADDRESS, 14, true);  //Get gyro, temp and accelerometer data

  if (14U == bytesReceived)
  {
    const uint8_t aXH = static_cast<uint8_t>(i2c.read());
    const uint8_t aXL = static_cast<uint8_t>(i2c.read());
    const uint8_t aYH = static_cast<uint8_t>(i2c.read());
    const uint8_t aYL = static_cast<uint8_t>(i2c.read());
    const uint8_t aZH = static_cast<uint8_t>(i2c.read());
    const uint8_t aZL = static_cast<uint8_t>(i2c.read());
    const int16_t rawA_X = (static_cast<int16_t>(aXH) << 8) | static_cast<int16_t>(aXL);
    const int16_t rawA_Y = (static_cast<int16_t>(aYH) << 8) | static_cast<int16_t>(aYL);
    const int16_t rawA_Z = (static_cast<int16_t>(aZH) << 8) | static_cast<int16_t>(aZL);

    const uint8_t tH = static_cast<uint8_t>(i2c.read());
    const uint8_t tL = static_cast<uint8_t>(i2c.read());
    data->temperature = (static_cast<int16_t>(tH) << 8) | static_cast<int16_t>(tL);

    const uint8_t gXH = static_cast<uint8_t>(i2c.read());
    const uint8_t gXL = static_cast<uint8_t>(i2c.read());
    const uint8_t gYH = static_cast<uint8_t>(i2c.read());
    const uint8_t gYL = static_cast<uint8_t>(i2c.read());
    const uint8_t gZH = static_cast<uint8_t>(i2c.read());
    const uint8_t gZL = static_cast<uint8_t>(i2c.read());
    const int16_t rawG_X = (static_cast<int16_t>(gXH) << 8) | static_cast<int16_t>(gXL);
    const int16_t rawG_Y = (static_cast<int16_t>(gYH) << 8) | static_cast<int16_t>(gYL);
    const int16_t rawG_Z = (static_cast<int16_t>(gZH) << 8) | static_cast<int16_t>(gZL);

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
* @brief    Reads a register data value.
* @param    Data representing the desired register address to read.
*/
uint8_t
Mpu6050::readRegister(const uint8_t theRegister)
{
  i2c.beginTransmission(MPU6050_I2C_ADDRESS);
  i2c.write(theRegister);   //Register
  i2c.endTransmission(false);
  i2c.requestFrom(MPU6050_I2C_ADDRESS, 1, true);  //Get gyro, temp and accelerometer data
  return static_cast<uint8_t>(i2c.read());
}
