/*
* MIT License
*
* Copyright (c) 2023 plainFlight
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

#ifndef DEFINES_H
#define DEFINES_H



//Debug defines - only uncomment one of these at one time to avoid chaos on serial terminal
//Note - If you have trouble programming the S3/C3 after enabling debug... unplug USB, hold down boot button and insert USB.
//#define SBUS_DEBUG 
//#define DEBUG_RADIO_COMMANDS
//#define DEBUG_GYRO_DATA
//#define DEBUG_PID
//#define DEBUG_GYRO_CALIBRATION
//#define DEBUG_MADGWICK
//#define DEBUG_BATTERY_VOLTS
//#define DEBUG_LOOP_RATE
//#define DEBUG_FLIGHT_STATE

/*
* Set the mixer to suit your model - or create a custom one
* Note: All mixes support 2 motor outputs that can if required be set for differential thrust.
*/
//#define MIXER_FLYING_WING                 //Elevons, rudder, throttle
//#define MIXER_PLANE_V_TAIL                //Left/right ailerons/flaps, Vtail elevator/rudder mix, throttle
#define MIXER_PLANE_FULL_HOUSE              //Left/right ailerons/flaps, elevator, rudder, throttle
//#define MIXER_PLANE_RUDDER_ELEVATOR       //Rudder, elevatore, throttle.


/*
* Uncomment if you want to use LED on Seeed XIAO ESP32S3 PCB
*/
//#define USE_LED_BUILTIN

/*
* External LED on LED_PIN can be sourced (driven high to light), or sinked (driven low to turn on)
*/
//#define SINK_LED  //Uncomment if your external LED port pin goes low to turn on LED

//If your IMU6050 is orientated facing towards the tail i.e. +X axis is towards the tail then uncomment the following...
#define MPU6050_Z_ROTATED_180

/*
* Servo refresh rate defines.
* Note: Only uncomment one of these at one time
* Note: Setting too higher rate could damage your servos - check servo manufacturers recomended settings
* Note: Due to how the ledc timer works, the lower the servo frequency the lower the step resolution of the timer PWM output
*/
//#define SERVO_REFRESH_50HZ   50     
//#define SERVO_REFRESH_100HZ  100     
#define SERVO_REFRESH_150HZ    150   
//#define SERVO_REFRESH_250HZ  250   
//#define SERVO_REFRESH_300HZ  300   

/*
* If using BLHeli ESC(s) then you can use oneshot125 prototcol to give 2KHz updates and 4096 motor steps
*/
//#define USING_ONESHOT125_ESC

/*
* Deadband settings - deending upon the quality of you tx you may need to set a deadband to stop the model drifting in rate mode
* Note: 5-10 is usually sufficient, but my RadioMaster Pocket has a problem with roll when at low throttle (mode 1 Tx), this has required a significant higher value (~25).
*/
#define TX_DEABAND_ROLL   25
#define TX_DEABAND_PITCH  25
#define TX_DEABAND_YAW    5

/*
* Servo trims.
* Set your model mechanically correct by flying in pass through and trimming control linkages.
* The servo trims should only really be used to set the servo horn centre position if the servo spline alignment is off slightly.
* Note: Signed integer value of LEDC timer ticks.
*/
#define TRIM_SERVO1 -50//-150
#define TRIM_SERVO2 -50//-150
#define TRIM_SERVO3 0
#define TRIM_SERVO4 0

/*
* Self levelled mode trimming.
* Trim IMU and/or flight controller mounting alignment issues with these trims.
* Values are in degrees.
*/
#define TRIM_LEVELLED_ROLL -1.5
#define TRIM_LEVELLED_PITCH 4.0f

/*
* If you find that gyro correction is working in the wrong sense then uncomment/comment the axis you need to reverse
*/
//#define REVERSE_ROLL_IMU
//#define REVERSE_PITCH_IMU
#define REVERSE_RUDDER_IMU

/*
* If you want to use differential throttle control with 2 motor then uncomment the following line...
*/
//#define USE_DIFFERENTIAL_THROTTLE

/*
* Max allowed demanded degrees per second by transmitter (degrees * 100)
* Note: If gyro set to 250 degs/s then do not exceed 230 degs/s (23000)
*/
#define MAX_ROLL_RATE_DEGS_x100     18000
#define MAX_PITCH_RATE_DEGS_x100    18000
#define MAX_YAW_RATE_DEGS_x100      10000
//Max angles allowed when in levelled mode (angles * 100)
#define MAX_ROLL_ANGLE_DEGS_x100    4500    
#define MAX_PITCH_ANGLE_DEGS_x100   4500

/*
* Fail safe flight angles.
* Recommend to set model to bank and pitch up slightly to gently spiral down.
* Note: Throttle is set low/off when in failsafe.
*/
#define FAILSAFE_ROLL_ANGLE         3.0f    //+1 degree x100 bank angle when in failsafe
#define FAILSAFE_PITCH_ANGLE        0.0f    //0 degree pitch angle when in failsafe

/*
* USB baud rate, can be lowered if required.
*/
#define USB_BAUD            500000

/*
* IO pin allocation.
* NOTE: You shouldn't need to change any of these unless you are making modifications to the code or target.
* NOTE: If using Seeed XIAO ESP32-C3 then set SBUS_TX_PIN to pin 25 (See below define).
*/
#define SERVO_1_PIN         D0
#define SERVO_2_PIN         D1
#define SERVO_3_PIN         D2
#define SERVO_4_PIN         D3
//#define IMU_SDA           D4
//#define IMU_SCL           D5
#define LED_PIN             D6  
#define SBUS_RX_PIN         D7
#define MOTOR_1_PIN         D8
#define MOTOR_2_PIN         D9
#define BATT_ADC_PIN        D10
/*Put Tx to an untracked spare IO pin to free up D6 (default UART Tx pin) for other usage*/
#define SBUS_TX_PIN         45    //Set to unused GPIO45 ESP32-S3
//#define SBUS_TX_PIN         25    //Set to unused GPIO18 ESP32-C3
#endif