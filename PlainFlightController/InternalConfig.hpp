#pragma once

#include "Config.hpp"
#include "CommonTypes.hpp"

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

  static_assert(NUMBER_SERVOS + NUMBER_MOTORS <= 8U, 
    "Configuration Error: Combined total of servos and motors cannot exceed 8.");

  // Battery telemetry transmit periods (milliseconds)
  static constexpr uint32_t TELEMETRY_BATTERY_PERIOD_MS      = 500U;  // 2Hz

  // Battery telemetry transmit periods (milliseconds)
  static constexpr uint32_t TELEMETRY_GNSS_PERIOD_MS      = 200U;  // 5Hz

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

  //==========================================================================
  // BUILD SETTINGS
  //==========================================================================

  // PlainFlightController firmware version string.
  static constexpr char SOFTWARE_VERSION[]                   = "V2.x.x";

  // USB serial baud rate and receiver UART port.
  static constexpr uint32_t USB_BAUD                         = 500000U;
  static constexpr HardwareSerial* const RECEIVER_UART       = &Serial0;
  static constexpr HardwareSerial* const GNSS_UART           = &Serial1;
}