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

#include "Defines.h"
#include "Radio_Ctrl.h"
#include "Actuators.h"

#define NUM_SERVOS          4
#define NUM_MOTORS          2
#define MAX_ACTUATORS       (NUM_SERVOS + NUM_MOTORS)
#define ONESHOT125_REFRESH  2000
#define LEDC_RESOLUTION     14


/*
* Check to see if we have set too many actuators for IO pins, if so throw compile time error.
*/
#if MAX_ACTUATORS > 6
#error Too many actuators, max allowed is 6.
#endif

static const uint8_t pwmPin[MAX_ACTUATORS] = {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN, MOTOR_1_PIN, MOTOR_2_PIN};



void initActuators(void)
{
  #if (NUM_SERVOS > 0)
    //Configure servo timer channels on ledc peripheral
    for(uint32_t i=0; i<NUM_SERVOS; i++)
    {
      ledcSetup(i, SERVO_REFRESH, LEDC_RESOLUTION);
      ledcAttachPin(pwmPin[i], i);
      ledcWrite(i, SERVO_CENTRE_TICKS);
    }
  #endif

  #if (NUM_MOTORS > 0)
    #ifdef USE_ONESHOT125_MOTORS
      //Configure motor timer channels on ledc peripheral for Oneshot125 protocol
      for(uint32_t i=NUM_SERVOS; i<(NUM_SERVOS+NUM_MOTORS); i++)
      {
        ledcSetup(i, ONESHOT125_REFRESH, LEDC_RESOLUTION);
        ledcAttachPin(pwmPin[i], i);
        ledcWrite(i, ONESHOT125_MIN_TICKS);
      }
    #else
      //Configure motor timer channels on ledc peripheral
      for(uint32_t i=NUM_SERVOS; i<(NUM_SERVOS+NUM_MOTORS); i++)
      {
        ledcSetup(i, SERVO_REFRESH, LEDC_RESOLUTION);
        ledcAttachPin(pwmPin[i], i);
        ledcWrite(i, SERVO_MIN_TICKS);
      }
    #endif
  #endif
}


/*
* DESCRIPTION: Writes required values to actuators that ae present.
* Note: Compile time #if's so we only write to actuators that were defined in initialise functions.
* This increases execution time compared to usng for loops.
*/
void setActuators(void)
{
  //Write required servos
  #if (NUM_SERVOS > 0)
   ledcWrite(0, actuator.servo1);
    #if (NUM_SERVOS > 1)
      ledcWrite(1, actuator.servo2);
      #if (NUM_SERVOS > 2)
        ledcWrite(2, actuator.servo3);
        #if (NUM_SERVOS > 3)
          ledcWrite(3, actuator.servo4);
        #endif
      #endif
    #endif
  #endif

  //Write required motors
  #if (NUM_MOTORS > 0)
    ledcWrite(4, actuator.motor1);
    #if (NUM_MOTORS > 1)
      ledcWrite(5, actuator.motor2);
    #endif
  #endif
}
