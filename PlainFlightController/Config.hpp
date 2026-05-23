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
* @file   Config.hpp
* @brief  Program-wide configuration for PlainFlightController.
*
* SECTION GUIDE
* -------------
* 1. BOARD & HARDWARE       - Define the physical ESP32S3 board.
* 2. RECEIVER               - Define the radio protocol.
* 3. MODEL TYPE             - Define the aircraft type.
* 4. OUTPUT ASSIGNMENT      - Define the output connection to servos/motors.
* 5. OUTPUT CONFIGURATION   - Change during physical installation (reversal, travel).
* 6. FEATURE FLAGS          - Enable/disable optional features for this build.
* 7. FLIGHT CHARACTERISTICS - Adjust during maiden flight and tuning.
* 8. MULTICOPTER SETTINGS   - Motor idle and minimum throttle, multicopter builds only.
* 9. BUILD & DEVELOPER      - Software version, debug flags. Most users should not need to edit this section.
*
*/

#pragma once

#include <cstdint>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "LedcServo.hpp"
#include "RxBase.hpp"
#include "BoardConfig.hpp"

/**
 * @class Config
 */
class Config
{
  public:

  //==========================================================================
  // SECTION 1: BOARD & HARDWARE SELECTION
  // Select the physical ESP32S3 board in use.
  // Available options (defined in BoardConfig.hpp): XIAO, ZERO, TINY
  //==========================================================================

  static constexpr BoardConfig::Board ESP32S3                = BoardConfig::XIAO;


  //==========================================================================
  // SECTION 2: RECEIVER
  // Select the receiver protocol matching your radio system.
  // Available options (defined in RxBase.hpp): CRSF, SBUS
  //==========================================================================

  static constexpr RxBase::ReceiverType RECEIVER_TYPE        = RxBase::ReceiverType::CRSF;


  //==========================================================================
  // SECTION 3: MODEL TYPE
  // Set exactly ONE model type to true; all others must be false.
  //==========================================================================

  static constexpr bool PLANE_FULL_HOUSE                     = true;
  static constexpr bool PLANE_FULL_HOUSE_V_TAIL              = false;
  static constexpr bool PLANE_ADVANCED_RUDDER_ELEVATOR       = false;
  static constexpr bool PLANE_RUDDER_ELEVATOR                = false;
  static constexpr bool PLANE_V_TAIL                         = false;
  static constexpr bool PLANE_FLYING_WING                    = false;
  static constexpr bool QUAD_X_COPTER                        = false;
  static constexpr bool QUAD_P_COPTER                        = false;
  static constexpr bool BI_COPTER                            = false;
  static constexpr bool CHINOOK_COPTER                       = false;
  static constexpr bool TRI_COPTER                           = false;
  static constexpr bool DUAL_COPTER                          = false;
  static constexpr bool SINGLE_COPTER                        = false;

  // Derived flags — do not edit. Used throughout the project to compile in/out
  // features specific to fixed-wing or multicopter model categories.
  // Note: if you add more model types, update both of these lines accordingly.
  static constexpr bool MODEL_IS_FIXED_WING                  = (PLANE_FULL_HOUSE || PLANE_FULL_HOUSE_V_TAIL || PLANE_ADVANCED_RUDDER_ELEVATOR || PLANE_RUDDER_ELEVATOR || PLANE_V_TAIL || PLANE_FLYING_WING);
  static constexpr bool MODEL_IS_MULTICOPTER                 = (QUAD_X_COPTER || QUAD_P_COPTER || BI_COPTER || CHINOOK_COPTER || TRI_COPTER || DUAL_COPTER || SINGLE_COPTER);


  //==========================================================================
  // SECTION 4: OUTPUT ASSIGNMENT
  // Maps the logical servo/motor channels of the selected model type to
  // physical output pins on the board.
  // - SERVO_PINS: listed in the order your servos are wired, first to last.
  // - MOTOR_PINS: listed in the order your motors are wired, first to last.
  // The number of entries in each array defines the servo/motor count.
  // You must update these arrays when changing model type.
  //
  // Example mappings by model:
  //   PlaneFullHouse:     4 servos, 2 motors
  //   TriCopter:          1 servo,  3 motors
  //   BiCopter:           2 servos, 2 motors
  //   PlaneRudderElev:    3 servos, 1 motor
  // See specific aircraft class in ModelTypes for further documentation
  // =========================================================================

  // PlaneFullHouse
  static constexpr uint8_t SERVO_PINS[] =
  {
      ESP32S3.OUTPUT_1,   // Servo 1 - e.g. left aileron  (PlaneFullHouse)
      ESP32S3.OUTPUT_2,   // Servo 2 - e.g. right aileron (PlaneFullHouse)
      ESP32S3.OUTPUT_3,   // Servo 3 - e.g. elevator      (PlaneFullHouse)
      ESP32S3.OUTPUT_4,   // Servo 4 - e.g. rudder        (PlaneFullHouse)
  };

  static constexpr uint8_t MOTOR_PINS[] =
  {
      ESP32S3.OUTPUT_5,   // Motor 1
      ESP32S3.OUTPUT_6,   // Motor 2
  };

  static constexpr uint8_t NUMBER_SERVOS = static_cast<uint8_t>(sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]));
  static constexpr uint8_t NUMBER_MOTORS = static_cast<uint8_t>(sizeof(MOTOR_PINS) / sizeof(MOTOR_PINS[0]));

  // Refresh rates for servos and motors.
  // Available options (defined in LedcServo.hpp):
  //   IS_50Hz, IS_100Hz, IS_150Hz, IS_200Hz, IS_250Hz, IS_300Hz, IS_350Hz, IS_ONESHOT125
  // Use IS_50Hz for analogue servos.
  // Use IS_ONESHOT125 for multicopter builds with BLHeli ESCs.
  // Note: higher refresh rates give better output resolution and flight controller response.
  // CAUTION: ensure your servos and ESCs are rated for the refresh rate you select.
  static constexpr LedcServo::RefreshRate SERVO_REFRESH_RATE = LedcServo::RefreshRate::IS_150Hz;
  static constexpr LedcServo::RefreshRate MOTOR_REFRESH_RATE = LedcServo::RefreshRate::IS_150Hz;


  //==========================================================================
  // SECTION 5: OUTPUT CONFIGURATION
  // Set during physical installation to match your wiring and servo orientation.
  //==========================================================================

  // Reverse individual outputs. Set the corresponding channel to true if a
  // servo moves in the wrong direction.
  static constexpr bool REVERSE_OUTPUT_1                     = false;
  static constexpr bool REVERSE_OUTPUT_2                     = false;
  static constexpr bool REVERSE_OUTPUT_3                     = false;
  static constexpr bool REVERSE_OUTPUT_4                     = false;
  static constexpr bool REVERSE_OUTPUT_5                     = false;
  static constexpr bool REVERSE_OUTPUT_6                     = false;
  static constexpr bool REVERSE_OUTPUT_7                     = false;
  static constexpr bool REVERSE_OUTPUT_8                     = false;

  // Extend servo travel beyond the standard 1.0–2.0 ms pulse range.
  // When true, travel is extended to 0.8–2.2 ms.
  static constexpr bool EXTEND_SERVO_TRAVEL_RANGE            = false;

  // Set to true to actively run the ESC calibration routine on next boot.
  // CAUTION: Remove all propellers before calibrating ESCs!
  // Set back to false after calibration is complete.
  static constexpr bool CALIBRATE_ESC                        = false;


  //==========================================================================
  // SECTION 6: FEATURE FLAGS
  // Enable or disable optional features for this aircraft build.
  //==========================================================================

  static constexpr bool USE_FLAPS                            = false;  // Flaps on Tx channel 7; use 2, 3 position switch or rotary pot.
  static constexpr bool USE_DIFFERENTIAL_THRUST              = false;  // Fixed-wing twin engine differential thrust.
  static constexpr bool USE_HEADING_HOLD                     = false;  // Gyro-based heading hold on Tx channel 8.
  static constexpr bool USE_LOW_VOLTS_CUT_OFF                = false;  // Limit throttle upon low battery voltage.
  static constexpr bool USE_EXTERNAL_LED                     = false;  // Enable external LED output.
  static constexpr bool USE_ACRO_TRAINER                     = false;  // Level mode when pitch & roll sticks centred, rate mode otherwise.

  // Prop hang / tail-sitter mode (experimental).
  // You need to understand flight controllers well to configure this.
  static constexpr bool USE_PROP_HANG_MODE                   = false;  // Self-levelling pitch/yaw for prop hanging via Tx channel 9.
  static constexpr bool PROP_HANG_TAIL_SITTER_MODE           = false;  // Roll stick commands yaw, yaw stick commands roll when prop hanging.


  //==========================================================================
  // SECTION 7: FLIGHT CHARACTERISTICS
  // Adjust these during maiden flight and subsequent tuning.
  //==========================================================================

  // Gyro rate range. Set exactly one to true.
  static constexpr bool USE_250_DEGS_SECOND                  = true;   // 250 degrees/second gyro rate.
  static constexpr bool USE_500_DEGS_SECOND                  = false;  // 500 degrees/second gyro rate.

  // Gyro correction direction. Set to true to reverse the correction sense on
  // the corresponding axis.
  static constexpr bool REVERSE_PITCH_CORRECTIONS            = false;
  static constexpr bool REVERSE_ROLL_CORRECTIONS             = false;
  static constexpr bool REVERSE_YAW_CORRECTIONS              = false;

  // Acro trainer recovery rate (degrees/second).
  // Caution: do not exceed the gyro rate set above.
  // Do not set too high on multicopters to avoid overshoot instability.
  static constexpr float ACRO_TRAINER_LEVEL_RATE             = 90.00f;

  // Prop hang correction direction. Adjust if corrections operate in the wrong sense.
  static constexpr bool REVERSE_PROP_HANG_PITCH_CORRECTIONS  = false;
  static constexpr bool REVERSE_PROP_HANG_YAW_CORRECTIONS    = false;
  static constexpr bool PROP_HANG_REVERSE_ROLL_DEMAND        = false;  // Set true if Tx roll stick demand is in the wrong sense in prop-hang mode.
  static constexpr bool PROP_HANG_REVERSE_YAW_DEMAND         = false;  // Set true if Tx yaw stick demand is in the wrong sense in prop-hang mode.

  // IMU board orientation corrections.
  // CAUTION: These are experimental and untested. Verify correct operation before flight.
  static constexpr bool IMU_ROLLED_RIGHT_90                  = false;  // IMU is rolled 90 degrees to the right of normal orientation.
  static constexpr bool IMU_ROLLED_180                       = false;  // IMU is flipped 180 degrees on the roll axis (upside down).

  // Transmitter stick deadband (normalised units, approximately 0.5% of normalised span).
  static constexpr uint32_t TX_DEADBAND_NORM                 = 10U;


  //==========================================================================
  // SECTION 8: MULTICOPTER MOTOR SETTINGS
  // Relevant for multicopter builds only.
  //==========================================================================

  // Motor idle speed when armed. Increase if motors stop mid-flight at low throttle;
  // decrease if idle RPM is too high.
  static constexpr int32_t IDLE_UP_VALUE                     = 300;

  // Absolute minimum motor command to prevent motors stopping under PID authority.
  // Adjust if minimum RPM is too low or too high.
  static constexpr int32_t MIN_THROTTLE_VALUE                = 100;


  //==========================================================================
  // SECTION 9: BUILD & DEVELOPER
  // Most users should not normally need to edit this section.
  //==========================================================================

  // PlainFlightController firmware version string.
  static constexpr char SOFTWARE_VERSION[]                   = "V2.x.x";

  // USB serial baud rate and receiver UART port.
  static constexpr uint32_t USB_BAUD                         = 500000U;
  static constexpr HardwareSerial* const RECEIVER_UART       = &Serial0;

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
};
