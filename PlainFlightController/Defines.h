/* 
* Copyright (c) 2023,2024 P.Cook (alias 'plainFlight')
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

#ifndef DEFINES_H
#define DEFINES_H

/*
* Defines for gains for rate, self-levelled and heading hold modes
* Note: Gain values will depend upon model type, control surface area and servo speed & refresh times.
*/
#define RATE_PITCH_P      100
#define RATE_PITCH_I      300
#define RATE_PITCH_D      0  
#define RATE_PITCH_F      18

#define RATE_ROLL_P       75
#define RATE_ROLL_I       150
#define RATE_ROLL_D       0  
#define RATE_ROLL_F       25

#define RATE_YAW_P        80
#define RATE_YAW_I        0  //Rudder i gain will fight an aileron turns, use heading hold functionality.
#define RATE_YAW_D        0  
#define RATE_YAW_F        45

#define LEVELLED_PITCH_P  140
#define LEVELLED_PITCH_I  0   //Be cautious with i gain on levelled mode.
#define LEVELLED_PITCH_D  0   
#define LEVELLED_PITCH_F  50  //Feed forward gives more stick strength but leads to max angle overshoot

#define LEVELLED_ROLL_P   140
#define LEVELLED_ROLL_I   0   //Be cautious with i gain on levelled mode.
#define LEVELLED_ROLL_D   0   
#define LEVELLED_ROLL_F   50  //Feed forward gives more stick strength but leads to max angle overshoot

#define HEADING_HOLD_P    120
#define HEADING_HOLD_I    300
#define HEADING_HOLD_D    0  
#define HEADING_HOLD_F    0

/*
* Debug defines - only uncomment one of these at one time to avoid chaos on serial terminal
* Note - If you have trouble programming the S3/C3 after enabling debug... unplug USB, hold down boot button and insert USB.
*/
//#define DEBUG_GYRO_CALIBRATION
//#define DEBUG_GYRO_DATA
//#define DEBUG_MADGWICK
//#define DEBUG_LOOP_RATE
//#define SBUS_DEBUG
//#define DEBUG_FLIGHT_STATE
//#define DEBUG_RADIO_COMMANDS
//#define DEBUG_BATTERY_VOLTS
//#define DEBUG_PID
//#define DEBUG_SERVO_MIXER
//#define DEBUG_MOTOR_MIXER

/*
* Set the mixer to suit your model - or create a custom one
* Note: All mixes support 2 motor outputs that can if required be set for differential thrust.
*/
//#define MIXER_FLYING_WING                 //Elevons, rudder, throttle
#define MIXER_PLANE_FULL_HOUSE              //Left/right ailerons/flaps, elevator, rudder, throttle
//#define MIXER_PLANE_FULL_HOUSE_V_TAIL     //Left/right ailerons/flaps, Vtail elevator/rudder mix, throttle
//#define MIXER_PLANE_RUDDER_ELEVATOR       //Rudder, elevatore, throttle.
//#define MIXER_PLANE_V_TAIL                //Vtail elevator/rudder mix, throttle

/*
* Select gyro rate for your model
* Note: GYRO_FS_SEL_250 will give better resolution for less agile aircraft.
* Note: Gains should not need changing if you decide to change at a later date.
*/
#define GYRO_FS_SEL_250     //Select for a more sedate aircraft or mild aerobatics.
//#define GYRO_FS_SEL_500   //Select for aerobatic aircraft.

/*
* Uncomment if you want to use LED on Seeed XIAO ESP32S3 PCB
* Note: On board LED (LED_BUILTIN) needs this uncommented.
* Note: Uncomment SINK_LED when using LED_BUILTIN.
*/
//#define USE_LED_BUILTIN

/*
* External LED on LED_PIN can be sourced (driven high to light), or sinked (driven low to turn on)
* Note: Uncomment if using LED_BUILTIN
*/
//#define SINK_LED  //Uncomment if your external LED port pin goes low to turn on LED

/*
* If you find that correction for rate and self-levelled modes are working in the wrong sense then uncomment/comment the axis you need to reverse
* Note: Depends on orientation of MPU6050 i.e. +X axis pointing towards nose or tail and also upon servo linkage hook up orientation.
*/
#define REVERSE_PITCH_CORRECTIONS
#define REVERSE_ROLL_CORRECTIONS
#define REVERSE_YAW_CORRECTIONS

/*
* Servo refresh rate defines.
* Note: Only uncomment one of these at one time
* Note: Setting too higher rate could damage your servos - check servo manufacturers recomended settings
* Note: Due to how the ledc timer works, the lower the servo frequency the lower the step resolution of the timer PWM output
*/
//#define SERVO_REFRESH_50HZ    50
//#define SERVO_REFRESH_100HZ   100
#define SERVO_REFRESH_150HZ     150
//#define SERVO_REFRESH_250HZ   250
//#define SERVO_REFRESH_300HZ   300

/*
* If using BLHeli ESC(s) then you can use oneshot125 prototcol to give 2KHz updates and 4096 motor steps
*/
//#define USE_ONESHOT125_ESC

/*
* Deadband settings - deending upon the quality of you tx you may need to set a deadband to stop the model drifting in rate mode
* Note: 5-10 is usually sufficient, but my RadioMaster Pocket has a problem with roll when at low throttle (mode 1 Tx), this has required a significant higher value (~25).
*/
#define TX_DEABAND_ROLL   25
#define TX_DEABAND_PITCH  25
#define TX_DEABAND_YAW    25

/*
* Servo trims.
* Set your model mechanically correct by flying in pass through and trimming control linkages.
* The servo trims should only really be used to set the servo horn centre position if the servo spline alignment is off slightly.
* Note: Signed integer value of LEDC timer ticks.
*/
#define TRIM_SERVO1   -50
#define TRIM_SERVO2   -50
#define TRIM_SERVO3   0
#define TRIM_SERVO4   0

/*
* Self levelled mode trimming.
* Trim IMU and/or flight controller mounting alignment issues with these trims.
* Values are in degrees.
*/
#define TRIM_LEVELLED_ROLL  -1.5f
#define TRIM_LEVELLED_PITCH 4.0f

/*
* If you want to use differential throttle control with 2 motor then uncomment the following line...
*/
//#define USE_DIFFERENTIAL_THROTTLE

/*
* USE_HEADING_HOLD_WHEN_YAW_CENTRED; If rudder is centred and Tx switch aux2 is enabled then i gain is added to the yaw PIDF calclation.
* USE_HEADING_HOLD; If Tx switch aux2 is enabled then i gain is added to the yaw PIDF calclation.
* Note: These are not desireable for normal flying as rudder would fight aileron turns. The idea is to turn it on and off as needed 
* i.e. on for run off ground take off, or during vertical manouvers to counter act wind or side thrust issues...
* If the model can prop-hang or knife edge then I have found USE_HEADING_HOLD to be best and will perform these maneuvers hands off once positioned in the correct attitude.
*/
#define USE_HEADING_HOLD
//#define USE_HEADING_HOLD_WHEN_YAW_CENTRED

/*
* Max allowed demanded degrees per second by transmitter (degrees * 100)
* Note: If gyro set to 250 degs/s then do not exceed 230 degs/s (23000)
*/
#define MAX_ROLL_RATE_DEGS_x100   18000
#define MAX_PITCH_RATE_DEGS_x100  18000
#define MAX_YAW_RATE_DEGS_x100    10000
//Max angles allowed when in levelled mode (angles * 100)
#define MAX_ROLL_ANGLE_DEGS_x100  4500
#define MAX_PITCH_ANGLE_DEGS_x100 4500

/*
* Fail safe flight angles.
* Recommend to set model to bank and pitch up slightly to gently spiral down.
* Note: Throttle is set low/off when in failsafe.
*/
#define FAILSAFE_ROLL_ANGLE   2.5f    //Degrees bank angle when in failsafe
#define FAILSAFE_PITCH_ANGLE  0.0f    //Degrees pitch angle when in failsafe

/*
* Enables flaps for 'full house' planes that have 2 aileron servos.
*/
#define USE_FLAPS                     //Comment out this line to disable flaps

/*
* Enable low battery throttle limiting and cut off.
* Use this is your ESC does not have a low voltage cut off to protect your flight pack.
*/
//#define USE_LOW_VOLT_CUT_OFF

/*
* USB baud rate, can be lowered if required.
*/
#define USB_BAUD 500000

/*
* IO pin allocation.
* NOTE: You shouldn't need to change any of these unless you are making modifications to the code or target.
* NOTE: If using Seeed XIAO ESP32-C3 then set SBUS_TX_PIN to pin 25 (See below define).
*/
#define SERVO_1_PIN   D0
#define SERVO_2_PIN   D1
#define SERVO_3_PIN   D2
#define SERVO_4_PIN   D3
//#define IMU_SDA     D4
//#define IMU_SCL     D5
#define LED_PIN       D6
#define SBUS_RX_PIN   D7
#define MOTOR_1_PIN   D8
#define MOTOR_2_PIN   D9
#define BATT_ADC_PIN  D10
/*Put Tx to an untracked spare IO pin to free up D6 (default UART Tx pin) for other usage*/
#define SBUS_TX_PIN   45  //Set to unused GPIO45 ESP32-S3
//#define SBUS_TX_PIN 25    //Set to unused GPIO18 ESP32-C3
#endif