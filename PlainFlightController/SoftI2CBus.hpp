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
* @file   SoftI2CBus.hpp
* @brief  Software I2C transport, shared by every IMU driver wired for I2C
*         (Mpu6050, Lsm6dsox, ...). Extracted from the identical transport code
*         each driver.
* @note   Method set (begin/writeRegister/readRegister/readRegisters) matches
*         SpiBus's, so a device driver can hold either as its DeviceBus with no
*         other code change.
*/
#pragma once

#include <cstdint>
#include <Arduino.h>
#include "ESP32_SoftWire.h"

class SoftI2CBus
{
  public:
    void begin(const uint8_t sdaPin, const uint8_t sclPin, const uint32_t clockHz, const uint8_t deviceAddress);
    void writeRegister(const uint8_t theRegister, const uint8_t theValue);
    uint8_t readRegister(const uint8_t theRegister);
    uint8_t readRegisters(const uint8_t startRegister, uint8_t* const buffer, const uint8_t length);

  private:
    uint8_t m_deviceAddress = 0U;

    //Objects
    SoftWire i2c;
};
