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
* @brief  This module contains structure that define different ESP32S3 boards.
*/
#pragma once


namespace BoardConfig
{

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
  const uint8_t RADIO_RECEIVER_RX;
  const uint8_t RADIO_RECEIVER_TX;
  const uint8_t LED_EXTERNAL;
  const uint8_t BATTERY_ADC;
};


/**
* @brief Structure representing the standard IO map for ESP32S3-XIAO.
*/
static constexpr Board ESP32S3_XIAO = 
{
  //LEDC channel pins used for servos/motors.
  .OUTPUT_1           = 1U, //GPIO1 = D0
  .OUTPUT_2           = 2U, //GPIO2 = D1
  .OUTPUT_3           = 3U, //GPIO3 = D2
  .OUTPUT_4           = 4U, //GPIO4 = D3
  .OUTPUT_5           = 7U, //GPIO7 = D8
  .OUTPUT_6           = 8U, //GPIO8 = D9
  //Other IO pins
  .LED_ON_BOARD       = 21U,//GPIO21
  .I2C_SDA            = 5U, //GPIO5 = D4
  .I2C_SCL            = 6U, //GPIO6 = D5
  .RADIO_RECEIVER_RX  = 44U,//GPIO44 = D7
  .RADIO_RECEIVER_TX  = 43U,//GPIO43 = D6???????????
  .LED_EXTERNAL       = 43U,//GPIO1 = D6?????????????
  .BATTERY_ADC        = 9U, //GPIO9 = D10
};


/**
* @brief Structure representing the standard IO map for ESP32S3-ZERO.
*/
static constexpr Board ESP32S3_ZERO = 
{
  //LEDC channel pins used for servos/motors.
  .OUTPUT_1           = 1U,   //GPIO1 
  .OUTPUT_2           = 2U,   //GPIO2
  .OUTPUT_3           = 3U,   //GPIO3
  .OUTPUT_4           = 4U,   //GPIO4
  .OUTPUT_5           = 5U,   //GPIO5
  .OUTPUT_6           = 6U,   //GPIO6
  .OUTPUT_7           = 14U,  //GPIO14
  .OUTPUT_8           = 15U,  //GPIO15
  //Other IO pins
  .LED_ON_BOARD       = 21U,  //GPIO21
  .I2C_SDA            = 12U,  //GPIO12
  .I2C_SCL            = 13U,  //GPIO13
  .RADIO_RECEIVER_RX  = 44U,  //GPIO44
  .RADIO_RECEIVER_TX  = 43U,  //GPIO43
  .LED_EXTERNAL       = 16U,  //GPIO16
  .BATTERY_ADC        = 7U,   //GPIO7
  //GPIO 8, 9, 10, 11, 17, 18, 38, 39, 40, 41, 42, 45 spare
};


/**
* @brief Structure representing the standard IO map for ESP32S3-TINY.
*/
static constexpr Board ESP32S3_TINY = 
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
  .LED_ON_BOARD       = 38U,   //GPIO38
  .I2C_SDA            = 18U,   //GPIO18
  .I2C_SCL            = 17U,   //GPIO17
  .RADIO_RECEIVER_RX  = 44U,   //GPIO44
  .RADIO_RECEIVER_TX  = 43U,   //GPIO43
  .LED_EXTERNAL       = 12U,   //GPIO12
  .BATTERY_ADC        = 13U,   //GPIO13
  //GPIO 9, 10, 11, 14, 15, 16, 21, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 45, 47, 48 spare
};

}//Namespace BoardConfig end.
