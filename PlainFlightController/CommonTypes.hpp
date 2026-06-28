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
* @file   CommonTypes.hpp
* @brief  This file contains enums and structures that are used throughout the program.
*/

#pragma once

#include <cstdint>

/**
* @enum ModelType
* @brief Defines all aircraft configurations supported by the flight controller.
*/
enum class ModelType : uint8_t
{
  PLANE_FULL_HOUSE,
  PLANE_FULL_HOUSE_V_TAIL,
  PLANE_ADVANCED_RUDDER_ELEVATOR,
  PLANE_RUDDER_ELEVATOR,
  PLANE_V_TAIL,
  PLANE_FLYING_WING,
  QUAD_X_COPTER,
  QUAD_P_COPTER,
  BI_COPTER,
  CHINOOK_COPTER,
  TRI_COPTER,
  DUAL_COPTER,
  SINGLE_COPTER
};

/**
* @enum ReceiverType
* @brief Supported radio protocols.
*/
enum class ReceiverType : uint8_t
{
  SBUS,
  CRSF
};

/**
* @enum GyroRate
* @brief Supported gyro rate range.
*/
enum class GyroRate : uint8_t
{
  IS_250_DEGS_SECOND,
  IS_500_DEGS_SECOND
};

/**
* @enum AircraftDir
* @brief Used for orientation of the gyro relative to the aircraft.
*        Note: The ordering is important here, it is used to ensure that 
*        a valid orientation is selected.
*/
enum class AircraftDir: uint8_t 
{ 
  FRONT, BACK, 
  LEFT, RIGHT, 
  UP, DOWN 
};

/**
* @enum RcChannelName
* @brief Standard channel names for RC control mapping.
* Other channels must be referred to by number
*/
enum class RcChannelName : uint32_t
{
    THROTTLE = 0U,
    ROLL,
    PITCH,
    YAW,
    ARM,
    MODE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6
};

/**
* @struct PassThroughStruct
* @brief Used for channel pass through structure in Config.hpp.
*/
struct PassThroughStruct
{
  uint8_t outputPin;
  RcChannelName channel;
};
