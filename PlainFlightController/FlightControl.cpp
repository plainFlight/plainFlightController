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
* @file   FlightControl.cpp
* @brief  This class contains methods to operate the flight controller.
*/

#include "FlightControl.hpp"



/**
* @brief  Starts the flight control process  
*/ 
void
FlightControl::begin()
{
  modelConfig();

  Serial.begin(Config::USB_BAUD);
  if (Serial)
  {
    delay(3000);  //Need to wait by this magic number or some console data will be lost while PC is connecting. 
    Serial.print("PlainFlightController: ");
    Serial.println(Config::SOFTWARE_VERSION);
  } 

  imu.begin();
  imu.setMadgwickWeighting(IMU::MADGWICK_WARM_UP_WEIGHTING);
  imuData = imu.getImuData();

  if (imu.isFaulted())
  {
    Serial.println("Faulted");
    m_flightState = DemandProcessor::FlightState::FAULTED;
  } 

  if (!config.begin())
  {
    Serial.println("Config not ok! rebooting...");
    delay(3000);
    ESP.restart();
  }

  myModel->begin();
  statusLed.begin();
  if constexpr(Config::USE_EXTERNAL_LED)
  {
    externLed.begin();
  }

  batteryMonitor.begin(config.getBatteryScaler());
}


/**
* @brief  Instantiates the model type  
*/ 
void
FlightControl::modelConfig()
{
  if constexpr(Config::PLANE_FULL_HOUSE)
  {
    myModel = new PlaneFullHouse();
  }
  else if constexpr(Config::PLANE_FULL_HOUSE_V_TAIL)
  {
    myModel = new PlaneFullHouseVTail();
  }
  else if constexpr(Config::PLANE_ADVANCED_RUDDER_ELEVATOR)
  {
    myModel = new PlaneAdvancedRudderElevator();
  }
  else if constexpr(Config::PLANE_RUDDER_ELEVATOR)
  {
    myModel = new PlaneRudderElevator();
  }  
  else if constexpr(Config::PLANE_V_TAIL)
  {
    myModel = new PlaneVTail();
  }   
  else if constexpr(Config::PLANE_FLYING_WING)
  {
    myModel = new PlaneFlyingWing();
  }   
  else if constexpr(Config::QUAD_X_COPTER)
  {
    myModel = new QuadXCopter();
  }  
  else if constexpr(Config::QUAD_P_COPTER)
  {
    myModel = new QuadPlusCopter();
  }  
  else if constexpr(Config::BI_COPTER)
  {
    myModel = new BiCopter();
  }  
  else if constexpr(Config::CHINOOK_COPTER)
  {
    myModel = new ChinookCopter();
  }   
  else if constexpr(Config::TRI_COPTER)
  {
    myModel = new TriCopter();
  }    
  else if constexpr(Config::DUAL_COPTER)
  {
    myModel = new DualCopter();
  }   
  else if constexpr(Config::SINGLE_COPTER)
  {
    myModel = new SingleCopter();
  }  
  else
  {
    static_assert(  (Config::PLANE_FULL_HOUSE ||
                    Config::PLANE_FULL_HOUSE_V_TAIL ||
                    Config::PLANE_ADVANCED_RUDDER_ELEVATOR ||
                    Config::PLANE_RUDDER_ELEVATOR ||
                    Config::PLANE_V_TAIL ||
                    Config::PLANE_FLYING_WING ||
                    Config::QUAD_X_COPTER ||
                    Config::BI_COPTER ||
                    Config::CHINOOK_COPTER ||
                    Config::TRI_COPTER ||
                    Config::DUAL_COPTER ||
                    Config::SINGLE_COPTER), 
                    "No model type selected by user. Set one in Config.hpp");
  }
}


/**
* @brief  Runs the flight control process  
*/ 
void 
FlightControl::operate()
{
  const float timedelta = loopRateControl();
  imu.operate(timedelta, &m_flightState);
  rc.process(&m_flightState, &m_lastFlightState, config.getRates(), config.getMaxAngles());
  checkStateChange();
  batteryMonitor.operate();
  statusLed.operate(static_cast<uint32_t>(m_flightState)); 

  if constexpr(Config::USE_EXTERNAL_LED)
  {
    externLed.operate(static_cast<uint32_t>(m_flightState)); 
  }

  if (!imu.isOk())
  {
    DemandProcessor::FlightState::FAULTED;
  }

  switch (m_flightState)
  {
    case DemandProcessor::FlightState::CALIBRATE:
      doCalibrateState();
      break;

    case DemandProcessor::FlightState::WAITING_TO_DISARM:
    case DemandProcessor::FlightState::DISARMED:
      doDisarmedState();
      break;

    default://Intentional fall through
    case DemandProcessor::FlightState::PASS_THROUGH:
      doPassThroughState();
      break;

    case DemandProcessor::FlightState::RATE:
      doRateState();
      break;

    case DemandProcessor::FlightState::FAILSAFE:
      doFailSafeState();
      break;

    case DemandProcessor::FlightState::SELF_LEVELLED:
      doLevelledState();
      break;
    
    case DemandProcessor::FlightState::ACRO_TRAINER:
      doAcroTrainerState();
      break;

    case DemandProcessor::FlightState::AP_WIFI:
      doWifiApState();
      break;

    case DemandProcessor::FlightState::FAULTED:
      doFaultedState();
      break;

    case DemandProcessor::FlightState::PROP_HANG:
      doPropHangState();
      break;
  }

  //Compile in/out debug data
  if constexpr(Config::DEBUG_RC_DATA)
  {
    rc.printData();
  }
  else if constexpr(Config::DEBUG_LOOP_RATE)
  {
    printLoopRateData();
  }
  else if constexpr(Config::DEBUG_BATTERY_MONITOR)
  {
    batteryMonitor.debug();
  }
  else
  {
    ;//Nothing compiled in
  }
}


/**
* @brief  Allows the gyro to calibrate and Madgwick filter to gain position.
*/ 
void 
FlightControl::doCalibrateState()
{
  if (imu.calibrateGyro())
  {
    //Complete so set madgwick to flight weighting...
    imu.setMadgwickWeighting(IMU::MADGWICK_FLIGHT_WEIGHTING);
    //Determine next state to go to...
    m_flightState = (rc.isArmed()) ? DemandProcessor::FlightState::WAITING_TO_DISARM : DemandProcessor::FlightState::DISARMED;
  }

  myModel->servoMixer(rc.getDemands(), config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);  //Ensure motors do not operate
}


/**
* @brief  Defines what is done when disarmed.
*/ 
void 
FlightControl::doDisarmedState()
{  
  myModel->servoMixer(rc.getDemands(), config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);  //Ensure motors do not operate
}


/**
* @brief  Defines what is done when in pass through mode.
*/ 
void 
FlightControl::doPassThroughState()
{
  DemandProcessor::Demands demands = *rc.getDemands();
  myModel->servoMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), SBus::MIN_NORMALISED_US);
  }

  myModel->motorMixer(&demands);
}


/**
* @brief  Defines what is done when in gyro rate mode.
*/ 
void 
FlightControl::doRateState()
{
  DemandProcessor::Demands demands = *rc.getDemands();
  processPIDF(&demands);
  myModel->servoRateMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), SBus::MIN_NORMALISED_US);
  }

  myModel->motorRateMixer(&demands);
}


/**
* @brief  Defines what is done when in self levelled mode.
*/ 
void
FlightControl::doFailSafeState()
{
  if constexpr(Config::MODEL_IS_MULTICOPTER)
  {
    //TODO - auto level & reduce throttle rather than just drop
    //For multicopters default demands to stop all motors from spinning.
    //Multicopters will fall out of the sky upon failsafe.
    myModel->servoMixer(&DemandProcessor::DEFAULT_DEMANDS, config.getServoTrims());
    myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);
  }
  else
  {
    doLevelledState();
  }
}


/**
* @brief  Defines what is done when in self levelled mode.
*/ 
void 
FlightControl::doLevelledState()
{  
  DemandProcessor::Demands demands = *rc.getDemands();

  //Angle demand is the error/difference between the stick demand and attitude of the model
  int32_t rollDemand = demands.roll - static_cast<int32_t>( ((imuData->roll + config.getRollTrim()) * 100.0f) );
  //Map error/difference from angle to degrees per second to produce roll rate
  rollDemand = map32(rollDemand, -config.getMaxRollAngle(),  config.getMaxRollAngle(), -config.getRollRate(), config.getRollRate());
  demands.roll = constrain(rollDemand, -config.getRollRate(), config.getRollRate());
  //Repeat calculations for pitch...
  int32_t pitchDemand = demands.pitch - static_cast<int32_t>( ((imuData->pitch + config.getPitchTrim()) * 100.0f) );
  pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(),  config.getMaxPitchAngle(), -config.getPitchRate(), config.getPitchRate());
  demands.pitch = constrain(pitchDemand, -config.getPitchRate(), config.getPitchRate());

  processPIDF(&demands);
  myModel->servoRateMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), SBus::MIN_NORMALISED_US);
  }
  
  myModel->motorRateMixer(&demands);
}


/**
* @brief  Defines what is done when in acro trainer mode.
* @note   When pitch and roll are centred we self level otherwise we operate in rate mode.
* @note   When sticks become centred the control system gets a step response, as a result we purposely use ACRO_TRAINER_LEVEL_RATE to limit...
* @note   ... how quickly we recover otherwise our normal flight gains can cause overshoot due to the step response. This overshoot can exceed...
* @note   ...the gyro degs/sec setting and clip the gyro response in turn screwing up the Madgwick filter level reference...
* @note   ...This is more of a problem with multicopters motors that can respond very quickly.
*/ 
void 
FlightControl::doAcroTrainerState()
{
  DemandProcessor::Demands demands = *rc.getDemands();

  if ((0 == demands.pitch) && (0 == demands.roll))
  {
    //Sticks are centred so self level
    DemandProcessor::Demands demands = *rc.getDemands();

    //Angle demand is the error/difference between the stick demand and attitude of the model
    int32_t rollDemand = demands.roll - static_cast<int32_t>( ((imuData->roll + config.getRollTrim()) * 100.0f) );
    //Map error/difference from angle to degrees per second to produce roll rate
    rollDemand = map32(rollDemand, -config.getMaxRollAngle(),  config.getMaxRollAngle(), -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);
    demands.roll = constrain(rollDemand, -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);

    //Repeat calculations for pitch...
    int32_t pitchDemand = demands.pitch - static_cast<int32_t>( ((imuData->pitch + config.getPitchTrim()) * 100.0f) );
    pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(),  config.getMaxPitchAngle(), -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);
    demands.pitch = constrain(pitchDemand, -ACRO_TRAINER_RECOVERY_RATE, ACRO_TRAINER_RECOVERY_RATE);

    processPIDF(&demands);
    myModel->servoRateMixer(&demands, config.getServoTrims());

    if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
    {
      demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), SBus::MIN_NORMALISED_US);
    }
    
    myModel->motorRateMixer(&demands);
  }
  else
  {
    doRateState();
  }
}


/**
* @brief  Defines what is done when in prop hanging state.
*/ 
void 
FlightControl::doPropHangState()
{
  DemandProcessor::Demands demands = *rc.getDemands();

  if constexpr(Config::PROP_HANG_TAIL_SITTER_MODE)
  {
    //Swap roll and yaw controls for prop hanging.
    int32_t rollDemand;
    
    if (Config::PROP_HANG_REVERSE_ROLL_DEMAND)
    {
      rollDemand = -demands.roll - static_cast<int32_t>( ((imuData->yaw + config.getYawTrim()) * 100.0f) );
    }
    else
    {
      rollDemand = demands.roll - static_cast<int32_t>( ((imuData->yaw + config.getYawTrim()) * 100.0f) );      
    }

    if constexpr(Config::PROP_HANG_REVERSE_YAW_DEMAND)
    {
      demands.yaw = -demands.yaw;
    }

    demands.roll = demands.yaw;
    //Angle demand is the error/difference between the stick demand and attitude of the model
    //Map error/difference from angle to degrees per second to produce roll rate
    rollDemand = map32(rollDemand, -config.getMaxRollAngle(), config.getMaxRollAngle(), -config.getYawRate(), config.getYawRate());
    demands.yaw = constrain(rollDemand, -config.getYawRate(), config.getYawRate());
    //Repeat calculations for pitch...
    int32_t pitchDemand = demands.pitch - static_cast<int32_t>( ((imuData->pitch + config.getPitchTrim()) * 100.0f) );
    pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(), config.getMaxPitchAngle(), -config.getPitchRate(), config.getPitchRate());
    demands.pitch = constrain(pitchDemand, -config.getPitchRate(), config.getPitchRate());
  }
  else
  {
    //Angle demand is the error/difference between the stick demand and attitude of the model
    int32_t yawDemand = demands.yaw - static_cast<int32_t>( ((imuData->yaw + config.getYawTrim()) * 100.0f) );
    //Map error/difference from angle to degrees per second to produce roll rate
    yawDemand = map32(yawDemand, -config.getMaxRollAngle(), config.getMaxRollAngle(), -config.getYawRate(), config.getYawRate());
    demands.yaw = constrain(yawDemand, -config.getYawRate(), config.getYawRate());
    //Repeat calculations for pitch...
    int32_t pitchDemand = demands.pitch - static_cast<int32_t>( ((imuData->pitch + config.getPitchTrim()) * 100.0f) );
    pitchDemand = map32(pitchDemand, -config.getMaxPitchAngle(), config.getMaxPitchAngle(), -config.getPitchRate(), config.getPitchRate());
    demands.pitch = constrain(pitchDemand, -config.getPitchRate(), config.getPitchRate());
  }

  processPIDF(&demands);
  myModel->servoRateMixer(&demands, config.getServoTrims());

  if constexpr(Config::USE_LOW_VOLTS_CUT_OFF)
  {
    demands.throttle = batteryMonitor.limitThrottle(demands.throttle, rc.throttleIsHigh(), SBus::MIN_NORMALISED_US);
  }
  
  myModel->motorRateMixer(&demands);
}


/**
* @brief  Defines what is done when in wifi configurator mode.
*/ 
void 
FlightControl::doWifiApState()
{
  myModel->servoMixer(&DemandProcessor::DEFAULT_DEMANDS, config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);

  batteryMonitor.setVoltageScaler(config.getBatteryScaler());
  config.updateBatteryVoltage(batteryMonitor.getVoltage());
  config.updateImuAngles((imuData->pitch + config.getPitchTrim()), (imuData->roll + config.getRollTrim()), (imuData->yaw + config.getYawTrim()));
  config.operate();
}


/**
* @brief  Defines what is done when faulted.
* @note   Only gets here from I2C read error, model will operate in pass through with no throttle.
* @note   If using mutlicopter then you are going to fall out of the sky !
*/ 
void 
FlightControl::doFaultedState()
{
  //bad things happened
  const DemandProcessor::Demands demands = *rc.getDemands();
  myModel->servoMixer(&demands, config.getServoTrims());
  myModel->motorMixer(&DemandProcessor::DEFAULT_DEMANDS);
}


/**
* @brief  When flight state changes i gain is zeroed out.
*/ 
void
FlightControl::checkStateChange()
{
  if (m_flightState != m_lastFlightState)
  {
    rollPIDF.iTermReset();
    pitchPIDF.iTermReset();
    yawPIDF.iTermReset();
    m_lastFlightState = m_flightState;
  } 
}


/**
* @brief  Processes pitch, roll and yaw PIDF's.
* @param  demands Structure of demand upon the system
* @param  roll  demand
* @param  yaw   demand
*/ 
void
FlightControl::processPIDF(DemandProcessor::Demands * const demands)
{
  if constexpr(Config::REVERSE_PITCH_CORRECTIONS)
  {
    demands->pitch = pitchPIDF.pidfController(demands->pitch, static_cast<int32_t>(-imuData->mpu6050.gyro_Y * 100.0f), config.getPitchGains());
  }
  else
  {
    demands->pitch = pitchPIDF.pidfController(demands->pitch, static_cast<int32_t>(imuData->mpu6050.gyro_Y * 100.0f), config.getPitchGains());
  }

  if constexpr(Config::REVERSE_ROLL_CORRECTIONS)
  {
    demands->roll = rollPIDF.pidfController(demands->roll, static_cast<int32_t>(-imuData->mpu6050.gyro_X * 100.0f), config.getRollGains());
  }
  else
  {
    demands->roll = rollPIDF.pidfController(demands->roll, static_cast<int32_t>(imuData->mpu6050.gyro_X * 100.0f), config.getRollGains());
  }

  float gyro_Z = imuData->mpu6050.gyro_Z;

  if constexpr(Config::REVERSE_YAW_CORRECTIONS)
  {
    //Purposely change sign of Z axis here to avoid screwing up madgwick filter.
    gyro_Z = -gyro_Z;
  }

  if constexpr(Config::MODEL_IS_MULTICOPTER)
  {
    //Multicopter only
    demands->yaw = yawPIDF.pidfController(demands->yaw, static_cast<int32_t>(gyro_Z * 100.0f), config.getYawGains());
  }
  else
  {
    //Fixed wing only
    if constexpr(Config::USE_HEADING_HOLD)
    {
      if (rc.headingHoldActive() || rc.propHangActive())
      {
        //Apply i gain to yaw
        demands->yaw = yawPIDF.pidfController(demands->yaw, static_cast<int32_t>(gyro_Z * 100.0f), config.getYawGains());
      }
      else
      {   
        //Do not apply i gain to yaw   
        yawPIDF.iTermReset();
        PIDF::Gains yawGains = *config.getYawGains();
        yawGains.i = 0;
        demands->yaw = yawPIDF.pidfController(demands->yaw, static_cast<int32_t>(gyro_Z * 100.0f), &yawGains);
      }
    }
    else
    {
      //Never apply i gain to yaw
      yawPIDF.iTermReset();
      PIDF::Gains yawGains = *config.getYawGains();
      yawGains.i = 0;
      demands->yaw = yawPIDF.pidfController(demands->yaw, static_cast<int32_t>(gyro_Z * 100.0f), &yawGains);
    }
  }
}