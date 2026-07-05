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
* 2. RECEIVER               - Define the external communication protocols.
* 3. MODEL TYPE             - Define the aircraft type.
* 4. OUTPUT ASSIGNMENT      - Define the output connection to servos/motors.
* 5. OUTPUT CONFIGURATION   - Change during physical installation (reversal, travel).
* 6. FEATURE FLAGS          - Enable/disable optional features for this build.
* 7. OPTIONAL GPS           - Configure a CASIC or UBX GPS to be used for telemetry.
* 8. FLIGHT CHARACTERISTICS - Adjust during maiden flight and tuning.
* 9. MULTICOPTER SETTINGS   - Motor idle and minimum throttle, multicopter builds only.
*
*/

#pragma once

#include <cstdint>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "LedcServo.hpp"
#include "CommonTypes.hpp"
#include "BoardConfig.hpp"
#include "GnssTypes.hpp"

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

  static constexpr BoardConfig::Board ESP32S3                = BoardConfig::ZERO;


  //==========================================================================
  // SECTION 2: RECEIVER
  // Select the protocols matching your radio system.
  // Receiver options (CommonTypes.hpp): CRSF, SBUS
  //==========================================================================

  static constexpr ReceiverType RECEIVER_TYPE                = ReceiverType::CRSF;

  //==========================================================================
  // SECTION 3: MODEL TYPE
  // Select exactly ONE model type.
  // Available options (defined in CommonTypes.hpp) PLANE_FULL_HOUSE, PLANE_FULL_HOUSE_V_TAIL,
  // PLANE_ADVANCED_RUDDER_ELEVATOR, PLANE_RUDDER_ELEVATOR, PLANE_V_TAIL, PLANE_FLYING_WING,
  // QUAD_X_COPTER, QUAD_P_COPTER, BI_COPTER, CHINOOK_COPTER, TRI_COPTER, DUAL_COPTER, SINGLE_COPTER
  //==========================================================================
  
  static constexpr ModelType MODEL_TYPE                      = ModelType::PLANE_FULL_HOUSE;

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
  //   PlaneFullHouse:              4 servos, 2 motors
  //   PlaneFullHouseVTail:         4 servos, 2 motors
  //   PlaneAdvancedRudderElevator: 2 servos, 2 motors
  //   PlaneRudderElevator:         2 servos, 2 motors
  //   PlaneVTail:                  2 servos, 2 motors
  //   PlaneFlyingWing:             3 servos, 2 motors
  //   QuadXCopter:                 0 servos, 4 motors
  //   QuadPlusCopter:              0 servos, 4 motors
  //   ChinookCopter:               2 servos, 2 motors
  //   BiCopter:                    2 servos, 2 motors
  //   TriCopter:                   1 servo,  3 motors
  //   DualCopter:                  2 servos, 2 motors
  //   SingleCopter:                4 servos, 1 motor
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
      // PlaneFullHouse expects two motors here even when not physically fitted 
      // as the code support differential thrust.
      ESP32S3.OUTPUT_5,   // Motor 1 - e.g. left/single motor  (PlaneFullHouse)
      ESP32S3.OUTPUT_6,   // Motor 2 - e.g. right motor (PlaneFullHouse)
  };

  // Refresh rates for servos, motors and any pass through channels.
  // Available options (defined in LedcServo.hpp):
  //   IS_50Hz, IS_100Hz, IS_150Hz, IS_200Hz, IS_250Hz, IS_300Hz, IS_350Hz, IS_ONESHOT125
  // Use IS_50Hz for analogue servos.
  // Use IS_ONESHOT125 for multicopter builds with BLHeli ESCs.
  // Note: higher refresh rates give better output resolution and flight controller response.
  // CAUTION: ensure your servos and ESCs are rated for the refresh rate you select.
  static constexpr LedcServo::RefreshRate SERVO_REFRESH_RATE = LedcServo::RefreshRate::IS_150Hz;
  static constexpr LedcServo::RefreshRate MOTOR_REFRESH_RATE = LedcServo::RefreshRate::IS_150Hz;
  static constexpr LedcServo::RefreshRate PASS_THROUGH_REFRESH_RATE = LedcServo::RefreshRate::IS_50Hz;

  // Any spare PWM channels not used by the model type can be used as channel pass through from Tx.
  // This can be useful to pass through functions such as under carriage and lights etc.
  // IMPORTANT NOTE: When using SBus if your model configuration uses more than 8 RC channels, then in Sbus.hpp set USE_ALL_18_CHANNELS = true 
  static constexpr PassThroughStruct PASS_THROUGH_PINS[] =
  {      
    //Output Pin to use,  Rx channel to assign
    {ESP32S3.OUTPUT_7,    RcChannelName::AUX3},  // E.g. Gear
    {ESP32S3.OUTPUT_8,    RcChannelName::AUX4}   // E.g. Lights
  };

  //==========================================================================
  // SECTION 5: OUTPUT CONFIGURATION
  // Set during physical installation to match your wiring and servo orientation.
  //==========================================================================

  // Reverse individual outputs. Last resort! Intended for correction of symmetric 
  // aileron/elevon servo installation and the like.  Ensure gyro orientation is 
  // correct and try reversing TX channel first.
  static constexpr bool REVERSE_OUTPUT_1                     = false;
  static constexpr bool REVERSE_OUTPUT_2                     = false;
  static constexpr bool REVERSE_OUTPUT_3                     = false;
  static constexpr bool REVERSE_OUTPUT_4                     = false;
  static constexpr bool REVERSE_OUTPUT_5                     = false;
  static constexpr bool REVERSE_OUTPUT_6                     = false;
  static constexpr bool REVERSE_OUTPUT_7                     = false;
  static constexpr bool REVERSE_OUTPUT_8                     = false;

  // Extend servo travel beyond the standard 1.0–2.0 ms pulse range.
  // Use when WiFi GUI trims have been used to offset servo centre position
  // in order to regain the full range of servo travel.
  // When true, travel is extended to 0.8–2.2 ms.
  static constexpr bool EXTEND_SERVO_TRAVEL_RANGE            = true;

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
  static constexpr bool HAS_TELEMETRY                        = true;   // Enable telemetry downlink to the RC transmitter.

  // Prop hang / tail-sitter mode (experimental).
  // You need to understand flight controllers well to configure this.
  static constexpr bool USE_PROP_HANG_MODE                   = false;  // Self-levelling pitch/yaw for prop hanging via Tx channel 9.
  static constexpr bool PROP_HANG_TAIL_SITTER_MODE           = false;  // Roll stick commands yaw, yaw stick commands roll when prop hanging.


  //==========================================================================
  // SECTION 7:  OPTIONAL GPS
  // Select the protocols matching your GPS hardware.
  // GNSS Type (CommonTypes.hpp):        NONE, CASIC, UBX
  // UBX Models (GnssTypes.hpp):         UBX_M6_MINUS, UBX_M7_M8, UBX_M9_PLUS
  // Note: dualGNSS library code is required to be installed even if GnssType::NONE is set. 
  // Note: Download the library from here: https://github.com/Cyberslug/dualGNSS
  //==========================================================================

  static constexpr GnssType     GNSS_TYPE                    = GnssType::NONE;
  static constexpr UbxSeries    GENERATION                   = UbxSeries::UBX_M6_MINUS; // Only relevant for GNSS_TYPE UBX


  //==========================================================================
  // SECTION 8: FLIGHT CHARACTERISTICS
  // Adjust these during maiden flight and subsequent tuning.
  // Available options (defined in CommonTypes.hpp) IS_250_DEG_SECOND, IS_500_DEG_SECOND
  //==========================================================================
  // CAUTION: The method of gyro orientation is new and untested. Verify correct operation before flight.

  // Gyro rate range. Set exactly one to true.
  static constexpr GyroRate GYRO_RATE                        = GyroRate::IS_250_DEGS_SECOND;

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

  // IMU board orientation.
  // IMU mounting is defined in terms of the direction the accelerometer plus_x and plus_y axes are facing
  // The x and y axis are typically marked on the module board.
  // Available options (defined in CommonTypes.hpp) are FRONT, BACK, LEFT, RIGHT, UP and DOWN
  static constexpr AircraftDir IMU_PLUS_X                    = AircraftDir::FRONT;  // Direction that the gyro plus x axis is facing.
  static constexpr AircraftDir IMU_PLUS_Y                    = AircraftDir::LEFT;   // Direction that the gyro plus y axis is facing.

  // Transmitter stick deadband (normalised units, approximately 0.5% of normalised span).
  static constexpr uint32_t TX_DEADBAND_NORM                 = 10U;

  
  //==========================================================================
  // SECTION 9: MULTICOPTER MOTOR SETTINGS
  // Relevant for multicopter builds only.
  //==========================================================================

  // Motor idle speed when armed. Increase if motors stop mid-flight at low throttle;
  // decrease if idle RPM is too high.
  static constexpr int32_t IDLE_UP_VALUE                     = 300;

  // Absolute minimum motor command to prevent motors stopping under PID authority.
  // Adjust if minimum RPM is too low or too high.
  static constexpr int32_t MIN_THROTTLE_VALUE                = 100;

};
