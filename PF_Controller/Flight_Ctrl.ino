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

#include "Flight_Ctrl.h"
#include "Radio_Ctrl.h"
#include "PIDF.h"
#include "IMU.h"
#include "Actuators.h"

extern void playLedSequence(states currentState);
states getRequiredState(states lastState);

//module variables
static states state;


/*
* DESCRIPTION: Main loop of plainFlight controller.
*/
void flightControl(void)
{
  states currentState;
  static states lastState = state_disarmed;
  int32_t roll_PIDF, pitch_PIDF, yaw_PIDF;

  getSbus();
  readIMUdata();
  Madgwick6DOF(imu.gyro_X, imu.gyro_Y, imu.gyro_Z, imu.accel_X, imu.accel_Y, imu.accel_Z, dt);
  batteryMonitor();
  currentState = getRequiredState(lastState);  
  processDemands(currentState);
  playLedSequence(currentState);

  if (lastState != currentState)
  {
    #ifdef DEBUG_FLIGHT_STATE
      Serial.print("State: ");
      Serial.println(currentState);
    #endif
    rollPIF.iTermReset();
    pitchPIF.iTermReset();
    yawPIF.iTermReset();
  } 

  switch(currentState)
  {
    case state_disarmed:  
      //If disarmed then operate in pass through mode and ensure throttle is at minimum.
      rxCommand.throttle = SERVO_MIN_TICKS;
    case state_pass_through:
      actuator.servo1 = rxCommand.roll;
      actuator.servo2 = rxCommand.roll;
      actuator.servo3 = rxCommand.pitch;
      actuator.servo4 = rxCommand.yaw;
      actuator.motor1 = rxCommand.throttle;
      actuator.motor2 = rxCommand.throttle;
      break;
    
    case state_rate:
      //Gyro based rate mode, control demands are in degrees/second x100.
      roll_PIDF = rollPIF.pidfController(   rxCommand.roll, GYRO_X, &gains[rate_gain].roll);
      pitch_PIDF = pitchPIF.pidfController( rxCommand.pitch,GYRO_Y, &gains[rate_gain].pitch);
      yaw_PIDF = yawPIF.pidfController(     rxCommand.yaw,  GYRO_Z, &gains[rate_gain].yaw);
       //Servos respond to deviations from required failsafe flight angle.
      actuator.servo1 = (uint32_t)map(roll_PIDF, -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.servo2 = (uint32_t)map(roll_PIDF, -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.servo3 = (uint32_t)map(pitch_PIDF,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.servo4 = (uint32_t)map(yaw_PIDF,  -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.motor1 = rxCommand.throttle;
      actuator.motor2 = rxCommand.throttle;
      break;

    default:
    case state_failsafe: 
      rxCommand.roll  = FAILSAFE_ROLL_ANGLE_x100;
      rxCommand.pitch = FAILSAFE_PITCH_ANGLE_x100;
      rxCommand.yaw = 0;
    case state_auto_level:
      //Gyro & accelerometer based Madgwick filter for levelled mode, control demands are in degrees x100.
      roll_PIDF = rollPIF.pidfController(   rxCommand.roll, (int32_t)((roll_IMU + trim.accRoll) * 100.0f), &gains[levelled_gain].roll);
      pitch_PIDF = pitchPIF.pidfController( rxCommand.pitch,(int32_t)((pitch_IMU + trim.accPitch) * 100.0f), &gains[levelled_gain].pitch); 
      //Yaw still works in rate mode, though you could use Madgwick output as heading hold function   
      yaw_PIDF = yawPIF.pidfController(     rxCommand.yaw,  GYRO_Z, &gains[rate_gain].yaw);  
      //Servos respond to deviations from required failsafe flight angle.
      actuator.servo1 = (uint32_t)map(roll_PIDF, -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.servo2 = (uint32_t)map(roll_PIDF, -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.servo3 = (uint32_t)map(pitch_PIDF,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.servo4 = (uint32_t)map(yaw_PIDF,  -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      actuator.motor1 = rxCommand.throttle;
      actuator.motor2 = rxCommand.throttle;
      //Set motor demand if not failsafe
      //TODO - do the oneshot stuff
      actuator.motor1 = (rxCommand.failsafe) ? SERVO_MIN_TICKS : actuator.motor1;
      actuator.motor2 = (rxCommand.failsafe) ? SERVO_MIN_TICKS : actuator.motor2;
      break;
  }
 
  //Add any trim offsets, or special functions (flaps) to servos
  actuator.servo1 += trim.servo1 - map(rxCommand.aux1Switch, (long)switch_low, (long)switch_high, 0, SERVO_HALF_TRAVEL_TICKS);
  actuator.servo2 += trim.servo2 + map(rxCommand.aux1Switch, (long)switch_low, (long)switch_high, 0, SERVO_HALF_TRAVEL_TICKS);
  actuator.servo3 += trim.servo3;
  actuator.servo4 += trim.servo4;
  //Adding trims, or very high gains may push us beyond servo operating range so constrain
  actuator.servo1 = constrain(actuator.servo1, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
  actuator.servo2 = constrain(actuator.servo2, SERVO_MIN_TICKS, SERVO_MAX_TICKS);  
  actuator.servo3 = constrain(actuator.servo3, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
  actuator.servo4 = constrain(actuator.servo4, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
  //Low battery voltage starts throttle limiting
  limitThrottle(&actuator.motor1, rxCommand.throttleIsLow);   
  limitThrottle(&actuator.motor2, rxCommand.throttleIsLow);
  //Update all actuators
  setActuators();
  lastState = currentState;
}



/*
* DESCRIPTION: Processes RC switch states to determine the require statemachine state.
* NOTE: Only allows arming if throttle is low.
* PARAMETERS:  The last flight state set.
* RETURNS:     The decoded state from set TX switches and SBUS failsafe.
*/
states getRequiredState(states lastState)
{  
  states requiredState;

  if (rxCommand.failsafe)
  {
    Serial.println("failsafe entered");///////////////////////////////////////////////////////////Removing this results in odd behaviour of this if statement??
    requiredState = state_failsafe;
  }
  else if (rxCommand.armSwitch)
  {  
    if ((rxCommand.throttleIsLow && (lastState == state_disarmed)) || (lastState != state_disarmed))
    {
      switch (rxCommand.modeSwitch)
      {
        case switch_low: 
          requiredState = state_pass_through;
          break;

        case switch_middle: 
          requiredState = state_rate;
          break;

        case switch_high: 
          requiredState = state_auto_level;
          break;
          
        default: 
          requiredState = state_failsafe;     //Should never get here but if we did then force failsafe and keep fingers crossed
          break;
      } 
    }   
  }
  else
  {
    requiredState = state_disarmed;
  }

  return requiredState;
}
