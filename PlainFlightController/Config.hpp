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
* @brief  This class contains program wide parameters used to configure operation of the PlainFlightController.
* @note   Contained within Configuration namespace to limit scope.
*/
#pragma once

#include <cstdint>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "LedcServo.hpp"



/**
 * @class Config
 */
class Config
{
  public:
    //Model type to instantiate, set one to true...
    static constexpr bool PLANE_FULL_HOUSE                    = true;
    static constexpr bool PLANE_FULL_HOUSE_V_TAIL             = false;
    static constexpr bool PLANE_ADVANCED_RUDDER_ELEVATOR      = false;
    static constexpr bool PLANE_RUDDER_ELEVATOR               = false;
    static constexpr bool PLANE_V_TAIL                        = false;
    static constexpr bool PLANE_FLYING_WING                   = false;
    static constexpr bool QUAD_X_COPTER                       = false;
    static constexpr bool QUAD_P_COPTER                       = false;
    static constexpr bool BI_COPTER                           = false;
    static constexpr bool CHINOOK_COPTER                      = false;
    static constexpr bool TRI_COPTER                          = false;
    static constexpr bool DUAL_COPTER                         = false;
    static constexpr bool SINGLE_COPTER                       = false;

    //Refresh rates of servos & motors, chose from the following but ensure you servos/ESC are capable of the rate set !
    //IS_50Hz, IS_100Hz, IS_150Hz, IS_200Hz, IS_250Hz, IS_300Hz, IS_350Hz, IS_ONESHOT125
    //If you are using analog servos use IS_50Hz. If configuring multicopter use IS_ONESHOT125 for BLHeli ESCs.
    //Note: higher refresh rates give better output resolution and flight controller reponse.
    static constexpr LedcServo::RefreshRate SERVO_REFRESH_RATE = LedcServo::RefreshRate::IS_150Hz;
    static constexpr LedcServo::RefreshRate MOTOR_REFRESH_RATE = LedcServo::RefreshRate::IS_150Hz;

    //Compile time configurable parameters/features
    static constexpr bool USE_FLAPS                           = false;  //Set to true for flaps on Tx channel 7, use 2, or 3 position switch, or rotary pot
    static constexpr bool USE_DIFFERENTIAL_THRUST             = false;  //Set to true for fixed wing twin engine differential thrust 
    static constexpr bool USE_HEADING_HOLD                    = false;  //Set to true for gyro based heading hold on Tx channel 8
    static constexpr bool USE_LOW_VOLTS_CUT_OFF               = false;  //Set to true to actively limit throttle upon low battery voltage
    static constexpr bool USE_250_DEGS_SECOND                 = true;   //Set to false for 250 degs/s
    static constexpr bool USE_500_DEGS_SECOND                 = false;  //Set to false for 500 degs/s
    static constexpr bool USE_EXTERNAL_LED                    = false; 
    static constexpr bool USE_ACRO_TRAINER                    = false;  //When pitch & roll sticks centred levelled mode, else rate mode. 
    static constexpr bool USE_ONBOARD_NEOPIXEL                = false;  //When using Waveshare ESP32-S3 Zero/Tiny set to true, make sure LED pin is set correctly.
    static constexpr bool REVERSE_PITCH_CORRECTIONS           = false;  //Set REVERSE_x_CORRECTIONS to true to reverse gyro/levelling corrections
    static constexpr bool REVERSE_ROLL_CORRECTIONS            = false;
    static constexpr bool REVERSE_YAW_CORRECTIONS             = false;
    static constexpr bool CALIBRATE_ESC                       = false;  //Remove all propellers before calibarting ESC's !   
    static constexpr bool REVERSE_SERVO_1                     = false;  //Should only need to use REVERSE_SERVO when same handed servos horns used on ailerons etc
    static constexpr bool REVERSE_SERVO_2                     = false;
    static constexpr bool REVERSE_SERVO_3                     = false;
    static constexpr bool REVERSE_SERVO_4                     = false;
    //Auto levelled prop hanging/tail sitting mode. You need to understand flight controllers well to configure this...
    static constexpr bool USE_PROP_HANG_MODE                  = false;  //Experimental mode that applies self levelling algorithm to pitch/yaw for prop hanging via Tx channel 9
    static constexpr bool REVERSE_PROP_HANG_PITCH_CORRECTIONS = false;  //Should pitch corrections operate in wrong sense in prophang then set to true
    static constexpr bool REVERSE_PROP_HANG_YAW_CORRECTIONS   = false;  //Should yaw corrections operate in wrong snese in prophang then set to true
    static constexpr bool PROP_HANG_TAIL_SITTER_MODE          = false;  //When true for tailsitter mode where roll stick commands models yaw, and yaw stick commands models roll when prop hanging.
    static constexpr bool PROP_HANG_REVERSE_ROLL_DEMAND       = false;  //If self leveling roll corrections are correct but Tx stick command in wrong sense then set to true.
    static constexpr bool PROP_HANG_REVERSE_YAW_DEMAND        = false;  //If self leveling yaw corrections are correct but Tx stick command in wrong sense then set to true.
    //Configure changes in board orientation...
    //CAUTION - These are experimental untested changes and need fully testing for correct operation before you fly!!
    static constexpr bool IMU_ROLLED_RIGHT_90                 = false;  //IMU is rolled to the right 90 degrees of normal orientation
    static constexpr bool IMU_ROLLED_180                      = false;  //IMU is rolled to 180 degrees of normal orientation i.e. flipped upside down on roll axis.

    //Acro trainer maximum recovery rate (level strength). 
    //Caution - do not exceed the set gyro degs/sec. Do not set to high for multicopters to avoid overshoot instability.
    static constexpr float ACRO_TRAINER_LEVEL_RATE            = 90.00f; //The degrees per second recovery rate.

    //Multicopter minimum motor speed settings
    static constexpr int32_t IDLE_UP_VALUE                    = 300;    //Motor idle speed when armed. Adjust this if idle up RPM is too low/high
    static constexpr int32_t MIN_THROTTLE_VALUE               = 100;    //Absolute minimum motor speed to prevent motors form stopping. Adjust this if idle up RPM is too low/high  

    //Tx stick deadband generally 5-15us
    static constexpr uint32_t TX_DEADBAND_US                  = 10U;

    //Debug constants that will allow debug data to be compiled in
    static constexpr bool DEBUG_SBUS                          = false;
    static constexpr bool DEBUG_RC_DATA                       = false; //Note: Disarmed and high to low throttle transition purposely resets ESP32 after Wifi, you will see this on console.
    static constexpr bool DEBUG_LOOP_RATE                     = false;
    static constexpr bool DEBUG_BATTERY_MONITOR               = false;
    static constexpr bool DEBUG_MADGWICK                      = false;
    static constexpr bool DEBUG_GYRO_CALIBRATION              = false;
    static constexpr bool DEBUG_CONFIGURATOR                  = false;
    static constexpr bool DEBUG_MPU6050                       = false;
    static constexpr bool DEBUG_MOTOR_OUTPUT                  = false;
    static constexpr bool DEBUG_SERVO_OUTPUT                  = false;
  
    //Auxillary IO pin allocation - only change if you know what you are doing
    //Note: As default pins D0, D1, D2, D3, D8, D9 are used for motors/servos.
    static constexpr uint8_t LED_ONBOARD                      = 21U;  //Pin 21 on XIAO or use LED_BUILTIN. Waveshare boards do not recognise LED_BUILTIN... Tiny is pin 38, Zero in pin 21.
    static constexpr uint8_t I2C_SDA                          = D4;
    static constexpr uint8_t I2C_SCL                          = D5;
    static constexpr uint8_t EXT_LED_PIN                      = D6;
    static constexpr uint8_t SBUS_RX                          = 44U;
    static constexpr uint8_t SBUS_TX                          = 43U;  //Pin function not used but reserved
    static constexpr uint8_t BATT_ADC_PIN                     = D10;

    //USB serial
    static constexpr uint32_t USB_BAUD                        = 500000U;
    static constexpr HardwareSerial * const SBUS_UART         = &Serial0; 

    //PlainFlightController build
    static constexpr char SOFTWARE_VERSION[]                  = "V2.0.1";   

    //Used to compile in/out features of different model categories
    //Note: if you add more model types make sure you add to these next 2 lines.
    static constexpr bool MODEL_IS_FIXED_WING                 = (PLANE_FULL_HOUSE || PLANE_FULL_HOUSE_V_TAIL || PLANE_ADVANCED_RUDDER_ELEVATOR || PLANE_RUDDER_ELEVATOR || PLANE_V_TAIL || PLANE_FLYING_WING); 
    static constexpr bool MODEL_IS_MULTICOPTER                = (QUAD_X_COPTER || QUAD_P_COPTER || BI_COPTER ||CHINOOK_COPTER || TRI_COPTER || DUAL_COPTER || SINGLE_COPTER); 
};
