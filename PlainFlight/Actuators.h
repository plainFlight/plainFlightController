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

#ifndef ACTUATORS_H
#define ACTUATORS_H

typedef struct
{
  int32_t servo1;
  int32_t servo2;
  int32_t servo3;
  int32_t servo4;
  int32_t motor1;
  int32_t motor2;
}Actuators;

Actuators actuator = {0};


/*
* Here we set the timer ticks for PWM depending upon the required servo refresh rate and motor protocol.
* Note: PWM counter is 14 bit or 16384 ticks.
* Ticks per ms = (16384/(1/SERVO_REFRESH_XXXHz))*0.001
* Rather than do this calculation at run time we have the folowing timer defines for different servo refresh rates
*/
//TODO - test all refresh rates and with flaps
#if defined(SERVO_REFRESH_300HZ)
  //4915 steps between 1-2ms
  #define SERVO_MIN_TICKS           4915
  #define SERVO_MAX_TICKS           9830
  #define SERVO_CENTRE_TICKS        7373
  #define SERVO_HALF_TRAVEL_TICKS   2458
  #define SERVO_REFRESH             300  
  #define SERVO_TRIM_MULTIPLIER     6  
#elif defined(SERVO_REFRESH_250HZ)
    //4096 steps between 1-2ms
    #define SERVO_MIN_TICKS           4096
    #define SERVO_MAX_TICKS           8192
    #define SERVO_CENTRE_TICKS        6144
    #define SERVO_HALF_TRAVEL_TICKS   2048
    #define SERVO_REFRESH             250 
    #define SERVO_TRIM_MULTIPLIER     5    
#elif defined(SERVO_REFRESH_150HZ)
      //2458 steps between 1-2ms
      #define SERVO_MIN_TICKS           2458
      #define SERVO_MAX_TICKS           4915
      #define SERVO_CENTRE_TICKS        3686
      #define SERVO_HALF_TRAVEL_TICKS   1229   
      #define SERVO_REFRESH             150 
      #define SERVO_TRIM_MULTIPLIER     3    
#elif defined(SERVO_REFRESH_100HZ) 
        //1638 steps between 1-2ms
        #define SERVO_MIN_TICKS           1638
        #define SERVO_MAX_TICKS           3277
        #define SERVO_CENTRE_TICKS        2458
        #define SERVO_HALF_TRAVEL_TICKS   819
        #define SERVO_REFRESH             100  
        #define SERVO_TRIM_MULTIPLIER     2   
#else
        //SERVO_REFRESH_50HZ
        //819 steps between 1-2ms
        #define SERVO_MIN_TICKS           819
        #define SERVO_MAX_TICKS           1638
        #define SERVO_CENTRE_TICKS        1229
        #define SERVO_HALF_TRAVEL_TICKS   410
        #define SERVO_REFRESH             50  
        #define SERVO_TRIM_MULTIPLIER     1   
#endif

//Oneshot125 max and min timer ticks
#define ONESHOT125_MIN_TICKS      4090
#define ONESHOT125_MAX_TICKS      8192

#if defined(USE_ONESHOT125_ESC)
  #define MOTOR_MIN_TICKS           ONESHOT125_MIN_TICKS
  #define MOTOR_MAX_TICKS           ONESHOT125_MAX_TICKS
#else
  #define MOTOR_MIN_TICKS           SERVO_MIN_TICKS
  #define MOTOR_MAX_TICKS           SERVO_MAX_TICKS
#endif

#endif