/* 
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
* @file   InternalConfig.hpp
* @brief  Header to hold internal configuration data that is used throughout the program.
*/

#pragma once

#include "Config.hpp"
#include "CommonTypes.hpp"
#include "ChannelValidation.hpp"

/**
 * @namespace InternalConfig
 * @brief Derived configuration flags and architectural logic.
 */
namespace InternalConfig
{
  static constexpr bool isFixedWing(ModelType type)
  {
    switch (type)
    {
      case ModelType::PLANE_FULL_HOUSE:
      case ModelType::PLANE_FULL_HOUSE_V_TAIL:
      case ModelType::PLANE_ADVANCED_RUDDER_ELEVATOR:
      case ModelType::PLANE_RUDDER_ELEVATOR:
      case ModelType::PLANE_V_TAIL:
      case ModelType::PLANE_FLYING_WING:
        return true;
      default:
        return false;
    }
  }

  static constexpr bool isMulticopter(ModelType type)
  {
    switch (type)
    {
      case ModelType::QUAD_X_COPTER:
      case ModelType::QUAD_P_COPTER:
      case ModelType::BI_COPTER:
      case ModelType::CHINOOK_COPTER:
      case ModelType::TRI_COPTER:
      case ModelType::DUAL_COPTER:
      case ModelType::SINGLE_COPTER:
        return true;
      default:
        return false;
    }
  }

  // Public architectural flags derived from user Config
  static constexpr bool MODEL_IS_FIXED_WING  = isFixedWing(Config::MODEL_TYPE);
  static constexpr bool MODEL_IS_MULTICOPTER = isMulticopter(Config::MODEL_TYPE);

  // Verify that every model is categorized
  static_assert(MODEL_IS_FIXED_WING != MODEL_IS_MULTICOPTER, 
    "Architectural Error: ModelType must be assigned to exactly ONE category (Fixed Wing or Multicopter).");

  // Derived counts for outputs
  static constexpr uint8_t NUMBER_SERVOS = 
    static_cast<uint8_t>(sizeof(Config::SERVO_PINS) / sizeof(Config::SERVO_PINS[0]));
  static constexpr uint8_t NUMBER_MOTORS = 
    static_cast<uint8_t>(sizeof(Config::MOTOR_PINS) / sizeof(Config::MOTOR_PINS[0]));
  static constexpr uint8_t NUMBER_PASS_THROUGH = 
    static_cast<uint8_t>(sizeof(Config::PASS_THROUGH_PINS) / sizeof(Config::PASS_THROUGH_PINS[0]));

  static_assert((NUMBER_SERVOS + NUMBER_MOTORS + NUMBER_PASS_THROUGH) <= 8U, 
    "Configuration Error: Combined total of servos, motors and pass through channels cannot exceed 8.");

  // Battery telemetry transmit periods (milliseconds)
  static constexpr uint32_t TELEMETRY_BATTERY_PERIOD_MS      = 500U;  // 2Hz

  // Battery telemetry transmit periods (milliseconds)
  static constexpr uint32_t TELEMETRY_GNSS_PERIOD_MS      = 200U;  // 5Hz

  // ARM and MODE assignment checks
  static_assert(Config::ARM_CHANNEL != RcChannelName::NONE,
    "Configuration Error: ARM_CHANNEL must be assigned to a channel.");
  static_assert(Config::MODE_CHANNEL != RcChannelName::NONE,
    "Configuration Error: MODE_CHANNEL must be assigned to a channel.");

  // Check channel assigned when feature is enabled
  static_assert(!Config::USE_FLAPS || (Config::FLAPS_CHANNEL != RcChannelName::NONE),
    "Configuration Error: USE_FLAPS is enabled but FLAPS_CHANNEL is not assigned.");
  static_assert(!Config::USE_HEADING_HOLD || (Config::HEADING_HOLD_CHANNEL != RcChannelName::NONE),
    "Configuration Error: USE_HEADING_HOLD is enabled but HEADING_HOLD_CHANNEL is not assigned.");
  static_assert(!Config::USE_PROP_HANG_MODE || (Config::PROP_HANG_CHANNEL != RcChannelName::NONE),
    "Configuration Error: USE_PROP_HANG_MODE is enabled but PROP_HANG_CHANNEL is not assigned.");

  // Check feature enabled when channel is assigned
  static_assert(Config::USE_FLAPS || (Config::FLAPS_CHANNEL == RcChannelName::NONE),
    "Configuration Error: FLAPS_CHANNEL is assigned but USE_FLAPS is not enabled.");
  static_assert(Config::USE_HEADING_HOLD || (Config::HEADING_HOLD_CHANNEL == RcChannelName::NONE),
    "Configuration Error: HEADING_HOLD_CHANNEL is assigned but USE_HEADING_HOLD is not enabled.");
  static_assert(Config::USE_PROP_HANG_MODE || (Config::PROP_HANG_CHANNEL == RcChannelName::NONE),
    "Configuration Error: PROP_HANG_CHANNEL is assigned but USE_PROP_HANG_MODE is not enabled.");

  // Duplicate channel assignment check 
  static_assert(!ChannelValidation::channelIsDuplicated(),
    "Configuration Error: Two or more RC functions and/or PassThrough are assigned to the same channel.");

  //==========================================================================
  // DEBUG SETTINGS
  //==========================================================================

  // Debug output flags. Setting any of these to true will compile in diagnostic
  // serial output for the corresponding subsystem. Leave all false for normal use.
  static constexpr bool DEBUG_RX                             = false;
  static constexpr bool DEBUG_RC_DATA                        = false;  // Note: disarmed high-to-low throttle transition intentionally resets ESP32 after WiFi.
  static constexpr bool DEBUG_LOOP_RATE                      = false;
  static constexpr bool DEBUG_BATTERY_MONITOR                = false;
  static constexpr bool DEBUG_MADGWICK                       = false;
  static constexpr bool DEBUG_GYRO_CALIBRATION               = false;
  static constexpr bool DEBUG_CONFIGURATOR                   = false;
  static constexpr bool DEBUG_MPU6050                        = false;
  static constexpr bool DEBUG_OUTPUT                         = false;
  static constexpr bool DEBUG_GPS                            = false;

  //==========================================================================
  // BUILD SETTINGS
  //==========================================================================

  // PlainFlightController firmware version string.
  static constexpr char SOFTWARE_VERSION[]                   = "V2.x.x";

  // USB serial baud rate and receiver UART port.
  static constexpr uint32_t USB_BAUD                         = 500000U;
  static constexpr HardwareSerial* const RECEIVER_UART       = &Serial0;
  static constexpr HardwareSerial* const GNSS_UART           = &Serial1;

}//Namespace InternalConfig end.
