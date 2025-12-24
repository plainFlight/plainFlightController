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

#include "Mpu6050.hpp"

/**
* @brief    Constructor that sets the desired gyro rate.
*/
Mpu6050::Mpu6050()
{
  if constexpr(Config::USE_250_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_250;
  }

  if constexpr(Config::USE_500_DEGS_SECOND)
  {
    m_scaleFactor = GYRO_SCALE_FACTOR_500;
  }
}


/**
* @brief    Initialises the MPU6050.
*/
void
Mpu6050::initialise()
{
  begin();
  reset();
  delay(50);

  setConfig(CONFIG);

  if constexpr(Config::USE_250_DEGS_SECOND)
  {
    setGyroConfig(GYRO_CONFIG_250);
  }

  if constexpr(Config::USE_500_DEGS_SECOND)
  {
    setGyroConfig(GYRO_CONFIG_500);
  }

  setAccelerometerConfig(ACCEL_CONFIG);  
}


/**
* @brief    Sets up and start the Wire I2C transfer.
*/
void 
Mpu6050::begin()
{
  Wire.setPins(Config::I2C_SDA,Config::I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLK_1MHZ);       //Overclocking, credit to drehmFlight code for this
}


/**
* @brief    Resets the MPU6050.
* @note     Reset initialises all registers to zero.
*/
void 
Mpu6050::reset()
{
  Wire.beginTransmission(MPU6050_ADD);
  Wire.write(0x6B);         //Register
  Wire.write(0x00);         //Data
  Wire.endTransmission(true);
}


/**
* @brief    Sets the mpu configuration.
* @param    Data representing the desired configuration register value.
*/
void 
Mpu6050::setConfig(const uint8_t config)
{
  Wire.beginTransmission(MPU6050_ADD);
  Wire.write(0x1A);         //Register
  Wire.write(config);       //Data
  Wire.endTransmission(true);
}


/**
* @brief    Sets the gyro scale to operate at.
* @param    Data representing the desired configuration register value.
*/
void 
Mpu6050::setGyroConfig(const uint8_t gyroScale)
{
  Wire.beginTransmission(MPU6050_ADD);
  Wire.write(0x1B);         //Register
  Wire.write(gyroScale);    //Data
  Wire.endTransmission(true);
}


/**
* @brief    Sets the accelerometer scale to operate at.
* @param    Data representing the desired configuration register value.
*/
void 
Mpu6050::setAccelerometerConfig(const uint8_t accelScale)
{
  Wire.beginTransmission(MPU6050_ADD);
  Wire.write(0x1C);         //Register
  Wire.write(accelScale);   //Data
  Wire.endTransmission(true);
}


/**
* @brief    Reads the gyro, temperature and accelerometer data form the mpu6050.
* @param    Pointer to data structure where mpu data is stored.
* @return   true when data successfully read.
*/
bool
Mpu6050::readData(MpuData* const data)
{
  Wire.beginTransmission(MPU6050_ADD);
  Wire.write(0x3B);                     //Register
  Wire.endTransmission(false);
  const uint8_t bytesReceived = Wire.requestFrom(MPU6050_ADD, 14, true);  //Get gyro, temp and accelerometer data

  if (14 == bytesReceived)
  {
    if constexpr(Config::IMU_ROLLED_RIGHT_90)
    {
      data->rawAccel_X = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawAccel_Z = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawAccel_Y = -(static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->temperature = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_X = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_Z = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_Y = -(static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
    }
    else if constexpr(Config::IMU_ROLLED_180)
    {
      data->rawAccel_X = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawAccel_Y = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawAccel_Z = -(static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->temperature = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_X = -(static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_Y = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_Z = -(static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
    }
    else
    {
      data->rawAccel_X = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawAccel_Y = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawAccel_Z = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->temperature = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_X = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_Y = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
      data->rawGyro_Z = (static_cast<int16_t>(Wire.read()) << 8) | static_cast<int16_t>(Wire.read());
    }

    data->gyro_X = static_cast<float>(data->rawGyro_X - data->gyroOffset_X) / m_scaleFactor;
    data->accel_X = static_cast<float>(data->rawAccel_X) / ACCEL_SCALE_FACTOR_16G;
    data->gyro_Y = static_cast<float>(data->rawGyro_Y - data->gyroOffset_Y) / m_scaleFactor;
    data->accel_Y = static_cast<float>(data->rawAccel_Y) / ACCEL_SCALE_FACTOR_16G;
    data->gyro_Z = static_cast<float>(data->rawGyro_Z - data->gyroOffset_Z) / m_scaleFactor;
    data->accel_Z = static_cast<float>(data->rawAccel_Z) / ACCEL_SCALE_FACTOR_16G;
  
    if constexpr(Config::DEBUG_MPU6050)
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

    return true;
  }
  else
  {
    if constexpr(Config::DEBUG_MPU6050)
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
  Wire.beginTransmission(MPU6050_ADD);
  Wire.write(theRegister);   //Register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADD, 1, true);  //Get gyro, temp and accelerometer data
  return Wire.read();
}


/**
* @brief    Gets the mpu device ID.
* @return   The device ID.
*/
uint8_t
Mpu6050::whoAmI()
{
  Wire.beginTransmission(MPU6050_ADD);
  Wire.write(0x75);         //Register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADD, 1, true);  //Get who am I data
  return Wire.read();
}
