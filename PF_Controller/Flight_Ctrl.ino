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
void modelMixer(Actuators *actuate, int32_t roll, int32_t pitch, int32_t yaw);
void motorMixer(Actuators *actuate, int32_t yaw);


/*
* DESCRIPTION: Main routine of plainFlight controller.
*/
void flightControl(void)
{
  states currentState;
  static states lastState = state_disarmed;
  int32_t roll_PIDF, pitch_PIDF, yaw_PIDF;
  Actuators control = {0};

  getSbus();
  readIMUdata();
  Madgwick6DOF(imu.gyro_X, imu.gyro_Y, imu.gyro_Z, imu.accel_X, imu.accel_Y, imu.accel_Z, timeDelta);
  batteryMonitor();
  currentState = getRequiredState(lastState);  
  processDemands(currentState);
  playLedSequence(currentState);

  if (lastState != currentState)
  {
    #if defined(DEBUG_FLIGHT_STATE)
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
      modelMixer(&control, rxCommand.roll, rxCommand.pitch, rxCommand.yaw);
      motorMixer(&control,rxCommand.yaw);
      break;
    
    case state_rate:
      //Gyro based rate mode, control demands are in degrees/second x100.
      roll_PIDF = rollPIF.pidfController(  rxCommand.roll, GYRO_X, &gains[rate_gain].roll);
      pitch_PIDF = pitchPIF.pidfController(rxCommand.pitch,GYRO_Y, &gains[rate_gain].pitch);
      #if defined(USE_HEADING_HOLD) || defined(USE_HEADING_HOLD_WHEN_YAW_CENTRED)
        headingHold(&yaw_PIDF);
      #else
        //Yaw still works in rate mode, though you could use Madgwick output as heading hold function 
        yaw_PIDF = yawPIF.pidfController(rxCommand.yaw,  GYRO_Z, &gains[rate_gain].yaw);
      #endif
      modelMixer(&control, roll_PIDF, pitch_PIDF, yaw_PIDF);
      motorMixer(&control, yaw_PIDF);
      //Convert PID demands to timer ticks for servo PWM/PPM
      control.servo1 = (int32_t)map(control.servo1,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      control.servo2 = (int32_t)map(control.servo2,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      control.servo3 = (int32_t)map(control.servo3,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      control.servo4 = (int32_t)map(control.servo4,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      break;

    default:
    case state_failsafe: 
      rxCommand.roll  = FAILSAFE_ROLL_ANGLE_x100;
      rxCommand.pitch = FAILSAFE_PITCH_ANGLE_x100;
      rxCommand.yaw = 0;
    case state_auto_level:
      //Gyro & accelerometer based Madgwick filter for levelled mode, control demands are in degrees x100.
      roll_PIDF = rollPIF.pidfController(rxCommand.roll, (int32_t)((imuRoll + trim.accRoll) * 100.0f), &gains[levelled_gain].roll);
      pitch_PIDF = pitchPIF.pidfController(rxCommand.pitch,(int32_t)((imuPitch + trim.accPitch) * 100.0f), &gains[levelled_gain].pitch); 
      #if defined(USE_HEADING_HOLD) || defined(USE_HEADING_HOLD_WHEN_YAW_CENTRED)
        headingHold(&yaw_PIDF);
      #else
        //Yaw still works in rate mode, though you could use Madgwick output as heading hold function 
        yaw_PIDF = yawPIF.pidfController(rxCommand.yaw,  GYRO_Z, &gains[rate_gain].yaw);
      #endif
      modelMixer(&control, roll_PIDF, pitch_PIDF, yaw_PIDF);
      motorMixer(&control, yaw_PIDF);
      //Convert PID demands to timer ticks for servo PWM/PPM
      control.servo1 = (int32_t)map(control.servo1,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      control.servo2 = (int32_t)map(control.servo2,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      control.servo3 = (int32_t)map(control.servo3,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      control.servo4 = (int32_t)map(control.servo4,-PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, SERVO_MIN_TICKS, SERVO_MAX_TICKS);
      break;
  }

  //Arguably flaps should be in modelMixer(), however, to obtain constant flap offsets it needs to be done after any map() functions of certain flight modes.
  #if defined(MIXER_PLANE_FULL_HOUSE) || defined(MIXER_PLANE_FULL_HOUSE_V_TAIL)
    uint32_t mappedFlaps = map(rxCommand.aux1Switch, (long)switch_low, (long)switch_high, 0, SERVO_HALF_TRAVEL_TICKS);
    control.servo1 -= mappedFlaps;
    control.servo2 += mappedFlaps;
  #endif

  //Add any trim offsets. Multiply by SERVO_TRIM_MULTIPLIER to normalise trim depending upon servo refresh rate due to changes in timer resolution.
  control.servo1 += trim.servo1 * SERVO_TRIM_MULTIPLIER;
  control.servo2 += trim.servo2 * SERVO_TRIM_MULTIPLIER;
  control.servo3 += trim.servo3 * SERVO_TRIM_MULTIPLIER;
  control.servo4 += trim.servo4 * SERVO_TRIM_MULTIPLIER;
  //Low battery voltage will start throttle limiting
  limitThrottle(&control.motor1, rxCommand.throttleIsLow);   
  limitThrottle(&control.motor2, rxCommand.throttleIsLow);
  //Update all actuators
  writeActuators(&control);
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
    Serial.println("failsafe entered");//TODO - ///////////Removing this results in odd behaviour of this if statement?? ...failsafe is magically entered briefly
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


/*
* DESCRIPTION: Mixes differential throttle if required, or pases through rx stick command for throttle value.
* NOTE: Also checks for failsafe and sets throttle to off if failsafe active.
*/
void motorMixer(Actuators *actuate, int32_t yaw)
{  
  //Convert PID/stick commands to timer ticks for servo PWM/PPM or Oneshot125
  #if defined(USE_DIFFERENTIAL_THROTTLE)
    int32_t mappedYaw = ((int32_t)map(yaw, -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT, MOTOR_MIN_TICKS, MOTOR_MAX_TICKS) * (int32_t)(DIFFERNTIAL_THRUST_GAIN * 100)) / 100;
    actuate->motor1 = (rxCommand.failsafe) ? MOTOR_MIN_TICKS : rxCommand.throttle + mappedYaw;
    actuate->motor2 = (rxCommand.failsafe) ? MOTOR_MIN_TICKS : rxCommand.throttle - mappedYaw;
  #else
    actuate->motor1 = (rxCommand.failsafe) ? MOTOR_MIN_TICKS : rxCommand.throttle;
    actuate->motor2 = actuator.motor1;
  #endif 
}


/*
* DESCRIPTION: Mixes control functions for the model type selected.
*/
void modelMixer(Actuators *actuate, int32_t roll, int32_t pitch, int32_t yaw)
{
  #if defined(MIXER_FLYING_WING)
    actuate->servo1 = roll - pitch;
    actuate->servo2 = roll + pitch;
    actuate->servo3 = yaw;
    actuate->servo4 = SERVO_CENTRE_TICKS;     //Spare Output
  #elif defined(MIXER_PLANE_FULL_HOUSE_V_TAIL) 
    actuate->servo1 = roll;
    actuate->servo2 = roll;
    actuate->servo3 = yaw + pitch;
    actuate->servo4 = yaw - pitch;
  #elif defined(MIXER_PLANE_FULL_HOUSE)
    actuate->servo1 = roll;
    actuate->servo2 = roll;
    actuate->servo3 = pitch;
    actuate->servo4 = yaw; 
  #elif defined(MIXER_PLANE_V_TAIL)
    actuate->servo1 = roll + pitch;
    actuate->servo2 = roll - pitch;
    actuate->servo3 = SERVO_CENTRE_TICKS;     //Spare Output;
    actuate->servo4 = SERVO_CENTRE_TICKS;     //Spare Output; 
  #elif defined(MIXER_PLANE_RUDDER_ELEVATOR)
    actuate->servo1 = roll;
    actuate->servo2 = pitch;
    actuate->servo3 = SERVO_CENTRE_TICKS;     //Spare Output
    actuate->servo4 = SERVO_CENTRE_TICKS;     //Spare Output
  #else
    #error No model MIXER defined by user ! 
  #endif
}


/*
* DESCRIPTION: Creates a heading hold feature for rudder equipt aircraft.
* Applies i gain to rudder when enabled to create a hold like function. Have not used Madwick yaw output as it is gyro based only and suffers from drift.
* As a result kept it simple by just adding i gain to rudder when required via Tx switch.
* Note: This Yaw PID output could be applied to roll with modification if you have no rudder available.
*/
void headingHold(int32_t* yaw)
{
  if (rxCommand.headingHold)
  {
    *yaw = yawPIF.pidfController(rxCommand.yaw,  GYRO_Z, &gains[levelled_gain].yaw);  
  }
  else
  {
    yawPIF.iTermReset();
    *yaw = yawPIF.pidfController(rxCommand.yaw,  GYRO_Z, &gains[rate_gain].yaw);  
  }
}
/*
void headingHold_Integrated(int32_t* yaw)
{
  static int32_t gyroSum = 0;

  if (rxCommand.headingHold)
  {
    gyroSum += GYRO_Z;
    *yaw = (gyroSum/1000)*-10;//(gyroSum/10000) * 300;//gains[levelled_gain].yaw.p;
    //Serial.println(*yaw);
  }
  else
  {
    yawPIF.iTermReset();
    gyroSum = 0;
    *yaw = yawPIF.pidfController(rxCommand.yaw,  GYRO_Z, &gains[rate_gain].yaw);  
  }
}*/