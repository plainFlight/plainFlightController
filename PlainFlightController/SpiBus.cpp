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
* @file   SpiBus.cpp
* @brief  Hardware SPI transport, for IMU drivers wired for SPI.
*/

#include "SpiBus.hpp"

/**
* @brief    Sets up the SPI peripheral (default SPIClass pins/bus - confirm against
*           the actual board before wiring real SPI hardware, see Spi_transport_
*           addition_plan.md section 4.4) and remembers the CS pin every subsequent
*           call addresses.
* @param    csPin  This board's IMU chip-select pin (Config::ESP32S3.IMU_SPI_CS).
*/
void
SpiBus::begin(const uint8_t csPin)
{
  m_csPin = csPin;
  pinMode(m_csPin, OUTPUT);
  digitalWrite(m_csPin, HIGH);  //Idle high - the device stays in SPI-idle/I2C-enabled state until a transaction below pulls CS low.
  spi.begin();
}


/**
* @brief    Writes data to a register.
* @param    theRegister representing the desired register address to write.
* @param    theValue the value to write.
*/
void
SpiBus::writeRegister(const uint8_t theRegister, const uint8_t theValue)
{
  spi.beginTransaction(SPISettings(SPI_CLK_10MHZ, MSBFIRST, SPI_MODE0));
  digitalWrite(m_csPin, LOW);
  spi.transfer(theRegister & static_cast<uint8_t>(~READ_BIT));  //R/W bit = 0 for a write.
  spi.transfer(theValue);
  digitalWrite(m_csPin, HIGH);
  spi.endTransaction();
}


/**
* @brief    Reads a single register.
* @param    theRegister representing the desired register address to read.
* @return   The register's value.
*/
uint8_t
SpiBus::readRegister(const uint8_t theRegister)
{
  spi.beginTransaction(SPISettings(SPI_CLK_10MHZ, MSBFIRST, SPI_MODE0));
  digitalWrite(m_csPin, LOW);
  spi.transfer(theRegister | READ_BIT);       //R/W bit = 1 for a read.
  const uint8_t value = spi.transfer(0x00U);  //Dummy byte, clocks out the device's reply.
  digitalWrite(m_csPin, HIGH);
  spi.endTransaction();
  return value;
}


/**
* @brief    Reads a sequential burst of registers, starting at startRegister, into buffer.
* @param    startRegister  First register address to read.
* @param    buffer         Destination for the bytes read - must hold at least length bytes.
* @param    length         Number of bytes to read.
* @return   Number of bytes read. Unlike I2C's requestFrom(), an SPI transfer() call has
*           no partial-completion signal - a caller getting fewer than length valid bytes
*           back is not a failure mode this transport can detect the way SoftI2CBus can,
*           so this always returns length once the transfer completes.
*/
uint8_t
SpiBus::readRegisters(const uint8_t startRegister, uint8_t* const buffer, const uint8_t length)
{
  spi.beginTransaction(SPISettings(SPI_CLK_10MHZ, MSBFIRST, SPI_MODE0));
  digitalWrite(m_csPin, LOW);
  spi.transfer(startRegister | READ_BIT);

  for (uint8_t i = 0U; i < length; i++)
  {
    buffer[i] = spi.transfer(0x00U);  //IF_INC (set in CTRL3_C by initialise()) auto-increments the register pointer between bytes.
  }

  digitalWrite(m_csPin, HIGH);
  spi.endTransaction();

  return length;
}
