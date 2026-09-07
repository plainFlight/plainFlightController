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
* @file   SoftI2CBus.cpp
* @brief  Software I2C transport, shared by every IMU driver wired for I2C.
*/

#include "SoftI2CBus.hpp"

/**
* @brief    Sets up and starts the SoftWire I2C transfer, and remembers the device
*           address every subsequent call addresses.
* @param    sdaPin, sclPin  I2C pins for this board (Config::ESP32S3.I2C_SDA/I2C_SCL).
* @param    clockHz         I2C bus clock.
* @param    deviceAddress   7-bit I2C address of the device this bus instance talks to.
*/
void
SoftI2CBus::begin(const uint8_t sdaPin, const uint8_t sclPin, const uint32_t clockHz, const uint8_t deviceAddress)
{
  m_deviceAddress = deviceAddress;
  i2c.begin(sdaPin, sclPin, clockHz);
  i2c.begin();
}


/**
* @brief    Writes data to a register.
* @param    theRegister representing the desired register address to write.
* @param    theValue the value to write.
*/
void
SoftI2CBus::writeRegister(const uint8_t theRegister, const uint8_t theValue)
{
  i2c.beginTransmission(m_deviceAddress);
  i2c.write(theRegister);   //Register
  i2c.write(theValue);      //Data
  i2c.endTransmission(true);
}


/**
* @brief    Reads a single register.
* @param    theRegister representing the desired register address to read.
* @return   The register's value.
*/
uint8_t
SoftI2CBus::readRegister(const uint8_t theRegister)
{
  i2c.beginTransmission(m_deviceAddress);
  i2c.write(theRegister);   //Register
  i2c.endTransmission(false);
  i2c.requestFrom(m_deviceAddress, 1, true);
  return static_cast<uint8_t>(i2c.read());
}


/**
* @brief    Reads a sequential burst of registers, starting at startRegister, into buffer.
* @param    startRegister  First register address to read.
* @param    buffer         Destination for the bytes read - must hold at least length bytes.
* @param    length         Number of bytes to read.
* @return   Number of bytes actually received - callers compare this against length to
*           detect a short/failed read, exactly as each driver's readData() already did
*           before this bus was extracted.
*/
uint8_t
SoftI2CBus::readRegisters(const uint8_t startRegister, uint8_t* const buffer, const uint8_t length)
{
  i2c.beginTransmission(m_deviceAddress);
  i2c.write(startRegister);              //Register
  i2c.endTransmission(false);
  const uint8_t bytesReceived = i2c.requestFrom(m_deviceAddress, length, true);

  for (uint8_t i = 0U; i < bytesReceived; i++)
  {
    buffer[i] = static_cast<uint8_t>(i2c.read());
  }

  return bytesReceived;
}
