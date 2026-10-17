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
* @brief    Sets up and starts this board's I2C bus.
* @note     Mpu6050 is I2C-only (see the static_assert in Mpu6050.hpp), so there is no
*           bus choice to make here - unlike Lsm6dsox::begin(), this always configures
*           the bus with this board's I2C pins and the device's fixed address.
*/
void
Mpu6050::begin()
{
  m_bus.begin(Config::ESP32S3.I2C_SDA, Config::ESP32S3.I2C_SCL, I2C_CLK_1MHZ, MPU6050_I2C_ADDRESS);
}


/**
* @brief    Writes data to a register.
* @param    theRegister representing the desired register address to write.
* @param    theValue the value to write.
*/
void
Mpu6050::writeRegister(const uint8_t theRegister, const uint8_t theValue)
{
  m_bus.writeRegister(theRegister, theValue);
}


/**
* @brief    Reads the gyro, temperature and accelerometer data form the mpu6050.
* @param    Pointer to data structure where mpu data is stored.
* @return   true when data successfully read.
*/
bool
Mpu6050::readData(ImuRawData* const data)
{
  uint8_t buffer[14];
  const uint8_t bytesReceived = m_bus.readRegisters(ACCEL_XOUT_H, buffer, 14U);  //Get gyro, temp and accelerometer data

  if (14U == bytesReceived)
  {
    const int16_t rawA_X = (static_cast<int16_t>(buffer[0]) << 8) | static_cast<int16_t>(buffer[1]);
    const int16_t rawA_Y = (static_cast<int16_t>(buffer[2]) << 8) | static_cast<int16_t>(buffer[3]);
    const int16_t rawA_Z = (static_cast<int16_t>(buffer[4]) << 8) | static_cast<int16_t>(buffer[5]);

    data->temperature = (static_cast<int16_t>(buffer[6]) << 8) | static_cast<int16_t>(buffer[7]);

    const int16_t rawG_X = (static_cast<int16_t>(buffer[8])  << 8) | static_cast<int16_t>(buffer[9]);
    const int16_t rawG_Y = (static_cast<int16_t>(buffer[10]) << 8) | static_cast<int16_t>(buffer[11]);
    const int16_t rawG_Z = (static_cast<int16_t>(buffer[12]) << 8) | static_cast<int16_t>(buffer[13]);

    finaliseImuSample(rawA_X, rawA_Y, rawA_Z, rawG_X, rawG_Y, rawG_Z,
                       m_scaleFactor, ACCEL_SCALE_FACTOR_16G, data);
    return true;
  }
  else
  {
    if constexpr(InternalConfig::DEBUG_IMU)
    {
      Serial.println("MPU6050 read error  !");
    }
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
  return m_bus.readRegister(theRegister);
}
