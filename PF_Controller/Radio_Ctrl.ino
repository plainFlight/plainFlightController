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

#include "Sbus_Rx.h"
#include "Flight_Ctrl.h"
#include "Defines.h"
#include "Radio_Ctrl.h"


//Module specific defines
#define SBUS_TIMEOUT  20000   //microseconds
//Enable SBUS_DEBUG then use serial monitor to find max and min SBUS values for your Tx... or adjust endpoints on Tx 
#define MAX_SBUS_US   1810
#define MIN_SBUS_US   172
#define MID_SBUS_US   991     //(MIN_SBUS_US + ((MAX_SBUS_US - MIN_SBUS_US) / 2))

//Arduino requires these declarations here for typedef's to work in function prototypes
void processDemands(states currentState);


/*
* DESCRIPTION: Grabs one SBUS packet, also detects loss of packets to set failsafe.
* Note: At power on the receiver may not output any data until connected with transmitter i.e. Tx failsafe set to no pulses.
*/
void getSbus(void) 
{
  static uint64_t sbusTimeout = 0;

  if (sbusRxMsg()) 
  {
    rxCommand.newSbusPacket = true;
    sbusTimeout = micros() + SBUS_TIMEOUT;

    #ifdef SBUS_DEBUG
    {
      // Display the received data 
      for (uint8_t i = 0; i < 16; i++) 
      {
        Serial.print(rxData.ch[i]);
        Serial.print("\t");
      }
      // Display lost frames and failsafe data 
      Serial.print(rxData.lost_frame);
      Serial.print("\t");
      Serial.println(rxData.failsafe);
    } 
    #endif
  }
  else if (micros() >= sbusTimeout)
  {
    //For this situation we need to detect and force failsafe flag.
    rxCommand.failsafe = true;
  }
}


/*
* DESCRIPTION: Takes SBUS data and scales it to bounds that we want to work with.
* For pitch, roll and yaw...
* When in pass through SBUS is scaled between 1ms and 2ms, bounds defined by MIN_SBUS_US and MAX_SBUS_US
* When in rate mode SBUS data is scales to degrees/second, bounds defined by MAX_XXXX_ANGLE_DEGS_x100.
* When in levelled mode SBUS data is scaled to a maximum pitch/roll angle, bounds defined by MAX_XXXX_ANGLE_DEGS_x100
* Note integer maths is used.
* For switch inputs, these are converted to states depending upon switch function.
* Throttle is effectively left in pass through, but scaled to suit PWM timer values. 
*/
void processDemands(states currentState)
{
  if (rxCommand.newSbusPacket)
  {
    rxCommand.newSbusPacket = false;

    if(state_auto_level == currentState)
    {
      //TODO - oneshot125 handleing
      rxCommand.throttle = map(rxData.ch[throttle],MIN_SBUS_US, MAX_SBUS_US, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      rxCommand.roll =  ((rxData.ch[roll] - MID_SBUS_US) > deadband.roll) ? map(rxData.ch[roll], MIN_SBUS_US, MAX_SBUS_US, -MAX_ROLL_ANGLE_DEGS_x100,  MAX_ROLL_ANGLE_DEGS_x100) : 0;
      rxCommand.pitch = ((rxData.ch[pitch] - MID_SBUS_US) > deadband.pitch) ? map(rxData.ch[pitch], MIN_SBUS_US, MAX_SBUS_US, -MAX_PITCH_ANGLE_DEGS_x100, MAX_PITCH_ANGLE_DEGS_x100) : 0;
      //Rudder still works in rate mode when level mode
      rxCommand.yaw =   ((rxData.ch[yaw] - MID_SBUS_US) > deadband.yaw) ? map(rxData.ch[yaw], MIN_SBUS_US, MAX_SBUS_US, -MAX_YAW_RATE_DEGS_x100, MAX_YAW_RATE_DEGS_x100) : 0;
    }
    else if(state_rate == currentState) 
    {
      //Following mapping reduces resolution from 11 bit to 10 bit to suit PWM/PPM outputs
      //TODO - oneshot125 handleing
      rxCommand.throttle = map(rxData.ch[throttle], MIN_SBUS_US, MAX_SBUS_US, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      rxCommand.roll =  ((rxData.ch[roll] - MID_SBUS_US) > deadband.roll) ? map(rxData.ch[roll],  MIN_SBUS_US, MAX_SBUS_US, -MAX_ROLL_RATE_DEGS_x100, MAX_ROLL_RATE_DEGS_x100) : 0;
      rxCommand.pitch = ((rxData.ch[pitch] - MID_SBUS_US) > deadband.pitch) ? map(rxData.ch[pitch], MIN_SBUS_US, MAX_SBUS_US, -MAX_PITCH_RATE_DEGS_x100,MAX_PITCH_RATE_DEGS_x100) : 0;
      rxCommand.yaw =   ((rxData.ch[yaw] - MID_SBUS_US) > deadband.yaw) ?  map(rxData.ch[yaw],   MIN_SBUS_US, MAX_SBUS_US, -MAX_YAW_RATE_DEGS_x100,  MAX_YAW_RATE_DEGS_x100) : 0;
    }
    else  //Pass through 
    {
      //TODO - oneshot125 handleing
      rxCommand.throttle = map(rxData.ch[throttle],MIN_SBUS_US, MAX_SBUS_US, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      rxCommand.roll = ((rxData.ch[roll] - MID_SBUS_US) > deadband.roll) ? map(rxData.ch[roll],   MIN_SBUS_US, MAX_SBUS_US, SERVO_MIN_TICKS, SERVO_MAX_TICKS) : SERVO_CENTRE_TICKS;
      rxCommand.pitch = ((rxData.ch[pitch] - MID_SBUS_US) > deadband.pitch) ? map(rxData.ch[pitch], MIN_SBUS_US, MAX_SBUS_US, SERVO_MIN_TICKS, SERVO_MAX_TICKS) : SERVO_CENTRE_TICKS;
      rxCommand.yaw = ((rxData.ch[yaw] - MID_SBUS_US) > deadband.yaw) ? map(rxData.ch[yaw],     MIN_SBUS_US, MAX_SBUS_US, SERVO_MIN_TICKS, SERVO_MAX_TICKS) : SERVO_CENTRE_TICKS;
    }

    //Channels 4 to 7 are uses as switch inputs, map to the required enum state
    //MISRA would have a fit at this :/ ...but hey its Arduino :)
    rxCommand.armSwitch =  (MID_SBUS_US < rxData.ch[aux1]) ? true : false;
    rxCommand.modeSwitch = (Switch_Mode)map(rxData.ch[aux2],   MIN_SBUS_US, MAX_SBUS_US, (long)pass_through, (long)levelled_mode);
    rxCommand.aux1Switch = (Switch_States)map(rxData.ch[aux3], MIN_SBUS_US, MAX_SBUS_US, (long)switch_low,   (long)switch_high);
    rxCommand.aux2Switch = (Switch_States)map(rxData.ch[aux4], MIN_SBUS_US, MAX_SBUS_US, (long)switch_low,   (long)switch_high);  
    //Copy failsafe flag
    rxCommand.failsafe = rxData.failsafe;
    rxCommand.throttleIsLow = (rxData.ch[throttle] < (MIN_SBUS_US + 100)) ? true : false;

    #ifdef DEBUG_RADIO_COMMANDS
      printRadioCommands();
    #endif
  }
}



/*
* DESCRIPTION: When DEBUG_RADIO_COMMANDS is defined data is printed to PC terminal.
*/
#ifdef DEBUG_RADIO_COMMANDS
  void printRadioCommands(void)
  {
    Serial.print("armed: ");
    Serial.print(rxCommand.armSwitch);
    Serial.print(", mode: ");
    Serial.print(rxCommand.modeSwitch);
    //Add other switches here as you need them
    Serial.print(", thr: ");
    Serial.print(rxCommand.throttle);
    Serial.print(", ail: ");
    Serial.print(rxCommand.roll);
    Serial.print(", pch: ");
    Serial.print(rxCommand.pitch);
    Serial.print(", yaw: ");
    Serial.println(rxCommand.yaw);
  }
#endif