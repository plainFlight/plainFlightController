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
* @file   BoardConfig.hpp
* @brief  This module contains structures that define different ESP32S3 boards.
*/
#pragma once

#include <cstdint>

namespace BoardConfig
{

/**
* @enum  ImuBus
* @brief Which bus a board is wired for the IMU on. A board that is physically fixed
*        to one bus (e.g. a dedicated carrier PCB) declares that bus here; a generic
*        dev board with no fixed IMU wiring declares the bus its default/shipped
*        wiring uses, and a user wiring that board differently edits this value (and
*        the matching IMU_SPI_CS field below) in their own copy of this board's entry.
*        The IMU's I2C address is not here - it's a device constant (e.g.
*        Lsm6dsox::LSM6DSOX_I2C_ADDRESS), the same way Mpu6050's fixed address is.
*/
enum class ImuBus : uint8_t
{
  I2C,
  SPI
};

/**
* @brief  Structure representing all IO used by Plain Flight Controller.
* @note   Some boards may have additional spare IO.
*/
struct Board
{
  //LEDC channel pins used for servos/motors.
  const uint8_t OUTPUT_1;
  const uint8_t OUTPUT_2;
  const uint8_t OUTPUT_3;
  const uint8_t OUTPUT_4;
  const uint8_t OUTPUT_5;
  const uint8_t OUTPUT_6;
  const uint8_t OUTPUT_7;
  const uint8_t OUTPUT_8;
  //Other IO pins
  const uint8_t LED_ON_BOARD;
  const uint8_t I2C_SDA;
  const uint8_t I2C_SCL;
  const uint8_t SERIAL_PORT_1_RX;
  const uint8_t SERIAL_PORT_1_TX;
  const uint8_t SERIAL_PORT_2_RX;
  const uint8_t SERIAL_PORT_2_TX;
  const uint8_t LED_EXTERNAL;
  const uint8_t BATTERY_ADC;
  //IMU transport - which bus this board is wired for. See ImuBus above and
  //local/Notes/Spi_transport_addition_plan.md.
  const ImuBus  IMU_BUS;          //Which bus the IMU uses on this board.
  const uint8_t IMU_SPI_CS;       //Meaningful when IMU_BUS == ImuBus::SPI, ignored otherwise.
  //Options
  const bool SINK_ONBOARD_LED;          //Set true to sink onboard LED, false to source onboard LED.
  const bool HAS_NEOPIXEL;              //When using a board with a Neopixel i.e. WS2812 or equivalent.
  const bool SWAP_NEOPIXEL_RGB_TO_GRB;  //If your neopixel ordering is green-red-blue then set this to true.
};


  /**
  * @brief Structure representing the standard IO map for ESP32S3-XIAO by Seed Studio.
  * @note Insufficient pins for GPS.
  */
  static constexpr Board XIAO =
  {
    //LEDC channel pins used for servos/motors.
    .OUTPUT_1           = 1U, //GPIO1 = D0
    .OUTPUT_2           = 2U, //GPIO2 = D1
    .OUTPUT_3           = 3U, //GPIO3 = D2
    .OUTPUT_4           = 4U, //GPIO4 = D3
    .OUTPUT_5           = 7U, //GPIO7 = D8
    .OUTPUT_6           = 8U, //GPIO8 = D9
    .OUTPUT_7           = 10U,//GPI10 = On J3 connector
    .OUTPUT_8           = 11U,//GPI11 = On J3 connector
    //Other IO pins
    .LED_ON_BOARD       = 21U,//GPIO21
    .I2C_SDA            = 5U, //GPIO5 = D4
    .I2C_SCL            = 6U, //GPIO6 = D5
    .SERIAL_PORT_1_RX   = 44U,//GPIO44 = D7
    .SERIAL_PORT_1_TX   = 45U,//GPIO45 = Unmapped pin on the XIAO. Remap if you want to use CRSF telemetry i.e. Try D6 if no external LED used.
    .SERIAL_PORT_2_RX   = 12U,//GPIO11 = On J3 connector
    .SERIAL_PORT_2_TX   = 13U,//GPIO12 = On J3 connector
    .LED_EXTERNAL       = 43U,//GPIO1 = D6
    .BATTERY_ADC        = 9U, //GPIO9 = D10
    //IMU transport - this is a generic dev board with no fixed IMU wiring; I2C is the
    //shipped default. XIAO has no documented spare GPIO (see "Insufficient pins for
    //GPS" above), so IMU_SPI_CS below is an UNCONFIRMED placeholder - do not wire SPI
    //on this board without first finding and testing a genuinely free pin.
    .IMU_BUS            = ImuBus::I2C,
    .IMU_SPI_CS         = 0U,   //UNCONFIRMED placeholder - see note above.
    //Options
    .SINK_ONBOARD_LED         = true,   //Set true to sink onboard LED, false to source onboard LED.
    .HAS_NEOPIXEL             = false,  //When using a board with a Neopixel i.e. WS2812 or equivalent.
    .SWAP_NEOPIXEL_RGB_TO_GRB = false,  //If your neopixel ordering is green-red-blue then set this to true.
  };


  /**
  * @brief Structure representing the standard IO map for ESP32S3-ZERO by Waveshare.
  */
  static constexpr Board ZERO =
  {
    //LEDC channel pins used for servos/motors.
    .OUTPUT_1           = 1U,   //GPIO1
    .OUTPUT_2           = 2U,   //GPIO2
    .OUTPUT_3           = 3U,   //GPIO3
    .OUTPUT_4           = 4U,   //GPIO4
    .OUTPUT_5           = 5U,   //GPIO5
    .OUTPUT_6           = 6U,   //GPIO6
    .OUTPUT_7           = 7U,   //GPIO7
    .OUTPUT_8           = 8U,   //GPIO8
    //Other IO pins
    .LED_ON_BOARD       = 21U,  //GPIO21
    .I2C_SDA            = 9U,   //GPIO9
    .I2C_SCL            = 10U,  //GPIO10
    .SERIAL_PORT_1_RX   = 44U,  //GPIO44
    .SERIAL_PORT_1_TX   = 43U,  //GPIO43
    .SERIAL_PORT_2_RX   = 11U,  //GPIO11
    .SERIAL_PORT_2_TX   = 12U,  //GPIO12
    .LED_EXTERNAL       = 14U,  //GPIO14
    .BATTERY_ADC        = 13U,  //GPIO13
    //GPIO 14, 15, 16, 17, 18, 38, 39, 40, 41, 42, 45 spare
    //IMU transport - this is a generic dev board with no fixed IMU wiring; I2C is the
    //shipped default. IMU_SPI_CS below is a placeholder from the spare GPIO list above,
    //not yet wired to any real hardware - confirm before use (see Spi_transport_
    //addition_plan.md section 4.4).
    .IMU_BUS            = ImuBus::I2C,
    .IMU_SPI_CS         = 15U,  //Placeholder spare GPIO, not used while IMU_BUS == I2C.
    //Options
    .SINK_ONBOARD_LED         = false,  //Set true to sink onboard LED, false to source onboard LED.
    .HAS_NEOPIXEL             = true,   //When using a board with a Neopixel i.e. WS2812 or equivalent.
    .SWAP_NEOPIXEL_RGB_TO_GRB = true,   //If your neopixel ordering is green-red-blue then set this to true.
  };


  /**
  * @brief Structure representing the standard IO map for ESP32S3-TINY by Waveshare.
  */
  static constexpr Board TINY =
  {
    //LEDC channel pins used for servos/motors.
    .OUTPUT_1           = 1U,   //GPIO1
    .OUTPUT_2           = 2U,   //GPIO2
    .OUTPUT_3           = 3U,   //GPIO3
    .OUTPUT_4           = 4U,   //GPIO4
    .OUTPUT_5           = 5U,   //GPIO5
    .OUTPUT_6           = 6U,   //GPIO6
    .OUTPUT_7           = 7U,   //GPIO7
    .OUTPUT_8           = 8U,   //GPIO8
    //Other IO pins
    .LED_ON_BOARD       = 38U,  //GPIO38
    .I2C_SDA            = 18U,  //GPIO18
    .I2C_SCL            = 17U,  //GPIO17
    .SERIAL_PORT_1_RX   = 44U,  //GPIO44
    .SERIAL_PORT_1_TX   = 43U,  //GPIO43
    .SERIAL_PORT_2_RX   = 11U,  //GPIO11
    .SERIAL_PORT_2_TX   = 12U,  //GPIO12
    .LED_EXTERNAL       = 14U,  //GPIO14
    .BATTERY_ADC        = 13U,  //GPIO13
    //GPIO 9, 10, 15, 16, 21, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 45, 47, 48 spare
    //IMU transport - this is a generic dev board with no fixed IMU wiring; I2C is the
    //shipped default. IMU_SPI_CS below is a placeholder from the spare GPIO list above,
    //not yet wired to any real hardware - confirm before use (see Spi_transport_
    //addition_plan.md section 4.4).
    .IMU_BUS            = ImuBus::I2C,
    .IMU_SPI_CS         = 15U,  //Placeholder spare GPIO, not used while IMU_BUS == I2C.
    //Options
    .SINK_ONBOARD_LED         = false,  //Set true to sink onboard LED, false to source onboard LED.
    .HAS_NEOPIXEL             = true,   //When using a board with a Neopixel i.e. WS2812 or equivalent.
    .SWAP_NEOPIXEL_RGB_TO_GRB = true,   //If your neopixel ordering is green-red-blue then set this to true.
  };

/**
* @brief Structure representing the standard IO map for the Waveshare module carrier board by Cyberslug.
*        Rev 0.0 is currently under test
*/
static constexpr Board WSMC =
{
  //LEDC channel pins used for servos/motors.
  .OUTPUT_1           = 1U,   //GPIO1
  .OUTPUT_2           = 2U,   //GPIO2
  .OUTPUT_3           = 3U,   //GPIO3
  .OUTPUT_4           = 4U,   //GPIO4
  .OUTPUT_5           = 5U,   //GPIO5
  .OUTPUT_6           = 6U,   //GPIO6
  .OUTPUT_7           = 7U,   //GPIO07
  .OUTPUT_8           = 8U,   //GPIO08
  //Other IO pins
  .LED_ON_BOARD       = 21U,  //GPIO21
  .I2C_SDA            = 9U,   //GPIO09
  .I2C_SCL            = 10U,  //GPIO10
  .SERIAL_PORT_1_RX   = 44U,  //GPIO44
  .SERIAL_PORT_1_TX   = 43U,  //GPIO43
  .SERIAL_PORT_2_RX   = 11U,  //GPIO11
  .SERIAL_PORT_2_TX   = 12U,  //GPIO12
  .LED_EXTERNAL       = 16U,  //GPIO16  not used
  .BATTERY_ADC        = 13U,  //GPIO13
  //GPIO 15, 17, 18, 38, 39, 40, 41, 42, 45 spare
  //IMU transport - this carrier board's module socket is hard-wired for I2C, with
  //SDO/SA0 strapped low (bench-confirmed, see Lsm6dsox_addition_plan.md section 3.7 -
  //matches Lsm6dsox::LSM6DSOX_I2C_ADDRESS's default, so no override is needed here).
  //There is no SPI option on this board.
  .IMU_BUS            = ImuBus::I2C,
  .IMU_SPI_CS         = 15U,  //Not used - this board is I2C-only.
  //Options
  .SINK_ONBOARD_LED         = false,  //Set true to sink onboard LED, false to source onboard LED.
  .HAS_NEOPIXEL             = true,   //When using a board with a Neopixel i.e. WS2812 or equivalent.
  .SWAP_NEOPIXEL_RGB_TO_GRB = true,  //If your neopixel ordering is green-red-blue then set this to true.
};

}//Namespace BoardConfig end.
