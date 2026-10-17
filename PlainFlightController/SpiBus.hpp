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
* @file   SpiBus.hpp
* @brief  Hardware SPI transport, for IMU drivers wired for SPI (currently
*         Lsm6dsox only). Framing/clock/mode below are sourced from ST's
*         LSM6DSOX datasheet.
* @note   Method set (begin/writeRegister/readRegister/readRegisters) matches
*         SoftI2CBus's, so a device driver can hold either as its DeviceBus
*         with no other code change. begin() takes only a CS pin, not a
*         device address - SPI addressing is done by the CS line, not a
*         bus-level address byte.
* @note   Default SPIClass peripheral/pins - confirm this against the actual
*         board before wiring real SPI hardware.
*/
#pragma once

#include <cstdint>
#include <Arduino.h>
#include <SPI.h>

class SpiBus
{
  public:
    static constexpr uint32_t SPI_CLK_10MHZ  = 10000000U;  //LSM6DSOX datasheet: 10 MHz max, register writes and burst reads alike.
    static constexpr uint8_t  READ_BIT       = 0x80U;      //First byte's R/W bit: 1 = read, 0 = write (LSM6DSOX datasheet).

    void begin(const uint8_t csPin);
    void writeRegister(const uint8_t theRegister, const uint8_t theValue);
    uint8_t readRegister(const uint8_t theRegister);
    uint8_t readRegisters(const uint8_t startRegister, uint8_t* const buffer, const uint8_t length);

  private:
    uint8_t m_csPin = 0U;

    //Objects
    SPIClass spi;
};
