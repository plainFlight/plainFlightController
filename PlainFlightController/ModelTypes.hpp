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
* @file   ModelTypes.hpp
* @brief  This class contains methods that all clases may call upon
*/
#pragma once

#include "LedcServo.hpp"
#include "RxBase.hpp"
#include "Utilities.hpp"
#include "PIDF.hpp"
#include "Config.hpp"
#include "BoardConfig.hpp"
#include "DemandProcessor.hpp"


/**
* @brief    Base class for all model types.
* @note     Purposely limited to 4 motors and 4 servos as this covers most model types.
* @note     Can be modified to use up to 8 servos, or 8 motors. Other combinations can be made.
* @note     If using different refresh rates for motors/servos then you must use even numbers as LEDc timers are shared between two channels i.e 0-1, 2-3, 4-5, 6-7. 
*/
class ModelBase : public Utilities
{
public:
  static constexpr uint8_t MAX_MOTORS = 4U;   //Must be even number as LEDc module shares 1 timer between 2 channels
  static constexpr uint8_t MAX_SERVOS = 4U;   //Must be even number as LEDc module shares 1 timer between 2 channels
  static constexpr uint8_t MAX_LEDC_CHANNELS = 8U;  //ESP32S3 has 8 PWM channels that operate in frequency pairs.

  //Structure used to define model type and actuators required.
  struct ModelConfig
  {
    uint8_t motorPins[MAX_MOTORS];
    uint8_t servoPins[MAX_SERVOS];
    LedcServo::RefreshRate motorRefresh;
    LedcServo::RefreshRate servoRefresh;
    uint8_t numberMotors;
    uint8_t numberServos;
  };

  ModelBase(ModelConfig modelConfig) : m_modelConfig(modelConfig)
  {
    assert((m_modelConfig.numberMotors <= MAX_MOTORS));  //Ensure not more that max number of motor channels not exceeded.
    assert((m_modelConfig.numberServos <= MAX_SERVOS));  //Ensure not more that max number of servo channels not exceeded.
    //Ensure not more that max number of LEDc channels have been decalred...
    //...Obviously the 2 asserts above would negate the need for the below, but just incase somebody tries to modify code beyond ESP32S3 HW limits.
    assert((m_modelConfig.numberMotors + m_modelConfig.numberServos) <= MAX_LEDC_CHANNELS);

    m_minServoTimerTicks = static_cast<int32_t>(servo[0].getMinTimerTicks());//All servos will be the same refresh rate
    m_maxServoTimerTicks = static_cast<int32_t>(servo[0].getMaxTimerTicks());
    m_minMotorTimerTicks = static_cast<int32_t>(motor[0].getMinTimerTicks());//And/or all motors will be the same reshresh rate
    m_maxMotorTimerTicks = static_cast<int32_t>(motor[0].getMaxTimerTicks());
  }

  ~ModelBase(){};

  /**
    * @brief  Initialises servos and motors
    */
  virtual void begin()
  {
    for (uint32_t i=0; i<m_modelConfig.numberServos; i++)
    {
      servo[i].begin();
    }

    for (uint32_t i=0; i<m_modelConfig.numberMotors; i++)
    {
      motor[i].begin();
    }

    if constexpr(Config::CALIBRATE_ESC)
    {
      for (uint32_t i=0; i<m_modelConfig.numberMotors; i++)
      {
        motor[i].setTimerTicks(motor[i].getMaxTimerTicks());
      }

      delay(LedcServo::CALIBRATE_ESC_DELAY);

      for (uint32_t i=0; i<m_modelConfig.numberMotors; i++)
      {
        motor[i].setTimerTicks(motor[i].getMinTimerTicks());
      }

      Serial.println("Calibration complete. Disable CALIBRATE_ESC setting in Config.hpp.");
      while(1);
    }
  }

  //TODO - the following should be pure virtual as we could write to uninitialised LEDc channels !!

  /**
    * @brief  Servo mixer
    * @param  demands - The RC demands
    * @param  trim - The saved servo trim values
    * @note   Cannot be pure virtual as some multicopters do not have servos.
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim){}

  /**
    * @brief  Servo mixer
    * @param  demands - The RC demands
    * @param  trim - The saved servo trim values
    * @note   Cannot be pure virtual as some multicopters do not have servos.
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim){}

  /**
    * @brief  Motor mixer pure virtual - can be overriden.
    * @param  demands - The RC demands 
    */
  virtual void motorMixer(DemandProcessor::Demands const * const demands) = 0;

  /**
    * @brief  Motor mixer pure virtual - must be overriden.
    * @param  demands - The RC demands
    */
  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) = 0;

  /**
    * @brief  This multiplier allows servo trim values to be constant across all RefreshRate's.
    */
  int32_t getTrimMultiplier() const {return servo[0].getTrimMultiplier();}

private:
  //Variables
  int32_t m_minServoTimerTicks;
  int32_t m_maxServoTimerTicks;
  int32_t m_minMotorTimerTicks;
  int32_t m_maxMotorTimerTicks;
  ModelConfig m_modelConfig;
  uint64_t m_motorDebugUpdateTime = 0U;
  uint64_t m_servoDebugUpdateTime = 0U;

  //Objects
  LedcServo servo[MAX_SERVOS] =
      {
          LedcServo(m_modelConfig.servoPins[0], m_modelConfig.servoRefresh, LedcServo::MID_MICRO_SECONDS),
          LedcServo(m_modelConfig.servoPins[1], m_modelConfig.servoRefresh, LedcServo::MID_MICRO_SECONDS),
          LedcServo(m_modelConfig.servoPins[2], m_modelConfig.servoRefresh, LedcServo::MID_MICRO_SECONDS),
          LedcServo(m_modelConfig.servoPins[3], m_modelConfig.servoRefresh, LedcServo::MID_MICRO_SECONDS),
      };

  LedcServo motor[MAX_MOTORS] =
      {
          LedcServo(m_modelConfig.motorPins[0], m_modelConfig.motorRefresh, LedcServo::MIN_MICRO_SECONDS),
          LedcServo(m_modelConfig.motorPins[1], m_modelConfig.motorRefresh, LedcServo::MIN_MICRO_SECONDS),
          LedcServo(m_modelConfig.motorPins[2], m_modelConfig.motorRefresh, LedcServo::MIN_MICRO_SECONDS),
          LedcServo(m_modelConfig.motorPins[3], m_modelConfig.motorRefresh, LedcServo::MIN_MICRO_SECONDS),
      };

  void debugServos(const uint32_t servo1, const uint32_t servo2, const uint32_t servo3, const uint32_t servo4)
  {
    const uint64_t nowTime = millis();

    if (m_servoDebugUpdateTime <= nowTime)
    {
      Serial.print("servo1: ");
      Serial.print(servo1);
      Serial.print(", servo2: ");
      Serial.print(servo2);
      Serial.print(", servo3: ");
      Serial.print(servo3);
      Serial.print(", servo4: ");
      Serial.print(servo4);
      Serial.println();
      m_servoDebugUpdateTime = nowTime + 100U;
    }
  }

  void debugMotors(const uint32_t motor1, const uint32_t motor2, const uint32_t motor3, const uint32_t motor4)
  {
    const uint64_t nowTime = millis();

    if (m_motorDebugUpdateTime <= nowTime)
    {
      Serial.print("motor1: ");
      Serial.print(motor1);
      Serial.print(", motor2: ");
      Serial.print(motor2);
      Serial.print(", motor3: ");
      Serial.print(motor3);
      Serial.print(", motor4: ");
      Serial.print(motor4);
      Serial.println();
      m_motorDebugUpdateTime = nowTime + 100U;
    }
  }

protected:
  static constexpr int32_t IDLE_UP = RxBase::MIN_NORMALISED + Config::IDLE_UP_VALUE;
  static constexpr int32_t MIN_THROTTLE = RxBase::MIN_NORMALISED + Config::MIN_THROTTLE_VALUE;

  enum class Actuator : uint32_t
  {
    CHANNEL_1 = 0U,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
  };

  //Variables
  int32_t m_idleUp = 0U;
  int32_t m_minThrottle = 0U;

  /**
    * @brief  Writes the demanded value/position to the servos.
    * @param  
    */
  void writeServos(const uint32_t servo1, const uint32_t servo2)
  {
    if constexpr(Config::REVERSE_SERVO_1)
    {
      servo[0].setTimerTicks((m_maxServoTimerTicks - servo1) + m_minServoTimerTicks);
    }
    else
    {
      servo[0].setTimerTicks(servo1);
    }

    if constexpr(Config::REVERSE_SERVO_2)
    {
      servo[1].setTimerTicks((m_maxServoTimerTicks - servo2) + m_minServoTimerTicks);
    }
    else
    {
      servo[1].setTimerTicks(servo2);
    }

    if constexpr(Config::DEBUG_SERVO_OUTPUT)
    {
      debugServos(servo1, servo2, servo[3].getDefaultTimerTicks(), servo[4].getDefaultTimerTicks());
    }
  }

  /**
    * @brief  Writes the demanded value/position to the servos.
    * @param  
    */
  void writeServos(const uint32_t servo1, const uint32_t servo2, const uint32_t servo3, const uint32_t servo4)
  {
    if constexpr(Config::REVERSE_SERVO_1)
    {
      servo[0].setTimerTicks((m_maxServoTimerTicks - servo1) + m_minServoTimerTicks);
    }
    else
    {
      servo[0].setTimerTicks(servo1);
    }

    if constexpr(Config::REVERSE_SERVO_2)
    {
      servo[1].setTimerTicks((m_maxServoTimerTicks - servo2) + m_minServoTimerTicks);
    }
    else
    {
      servo[1].setTimerTicks(servo2);
    }

    if constexpr(Config::REVERSE_SERVO_3)
    {
      servo[2].setTimerTicks((m_maxServoTimerTicks - servo3) + m_minServoTimerTicks);
    }
    else
    {
      servo[2].setTimerTicks(servo3);
    }

    if constexpr(Config::REVERSE_SERVO_4)
    {
      servo[3].setTimerTicks((m_maxServoTimerTicks - servo4) + m_minServoTimerTicks);
    }
    else
    {
      servo[3].setTimerTicks(servo4);
    }

    if constexpr(Config::DEBUG_SERVO_OUTPUT)
    {
      debugServos(servo1, servo2, servo3, servo4);
    }
  }

  /**
    * @brief  Writes the required demanded values to the motors.
    * @param  
    */
  void writeMotors(const uint32_t motor1, const uint32_t motor2)
  {
    motor[0].setTimerTicks(motor1);
    motor[1].setTimerTicks(motor2);

    if constexpr(Config::DEBUG_MOTOR_OUTPUT)
    {
      debugMotors(motor1, motor2, motor[2].getDefaultTimerTicks(), motor[3].getDefaultTimerTicks());
    }
  }

  void writeMotors(const uint32_t motor1, const uint32_t motor2, const uint32_t motor3, const uint32_t motor4)
  {
    motor[0].setTimerTicks(motor1);
    motor[1].setTimerTicks(motor2);
    motor[2].setTimerTicks(motor3);
    motor[3].setTimerTicks(motor4);

    if constexpr(Config::DEBUG_MOTOR_OUTPUT)
    {
      debugMotors(motor1, motor2, motor3, motor4);
    }
  }

  /**
    * @brief  Converts calculated pass through servo demands to correct timer tick values.
    * @param  
    */
  uint32_t mapNormalisedServoToTimerTicks(const int32_t channelValue)
  {
    return static_cast<uint32_t>(map32(channelValue, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, m_minServoTimerTicks, m_maxServoTimerTicks));
  }

  /**
    * @brief  Converts calculated pass through motor demands to correct timer tick values.
    * @param  
    */
  uint32_t mapNormalisedMotorToTimerTicks(const int32_t channelValue)
  {
    return static_cast<uint32_t>(map32(channelValue, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, m_minMotorTimerTicks, m_maxMotorTimerTicks));
  }

  /**
    * @brief  Converts calculated rate mode motor demands to correct timer tick values.
    * @param  
    */
  uint32_t mapRateMotorToTimerTicks(const int32_t channelValue)
  {
    return static_cast<uint32_t>(map32(channelValue, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT, m_minMotorTimerTicks, m_maxMotorTimerTicks));
  }

  /**
    * @brief  Converts calculated rate mode servo demands to correct timer tick values.
    * @param  
    */
  uint32_t mapRateServoToTimerTicks(const int32_t channelValue)
  {
    return static_cast<uint32_t>(map32(channelValue, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT, m_minServoTimerTicks, m_maxServoTimerTicks));
  }

  /**
    * @brief  Gets the default timer ticks for motors i.e. motors off.
    * @param  
    */
  uint32_t getDefaultMotorTicks(Actuator number)
  {
    return motor[static_cast<uint32_t>(number)].getDefaultTimerTicks();
  }

  /**
    * @brief  Gets the default timer ticks for servos i.e. servo centred.
    * @param  
    */
  uint32_t getDefaultServoTicks(Actuator number)
  {
    return servo[static_cast<uint32_t>(number)].getDefaultTimerTicks();
  }

  /**
    * @brief  Gets the max timer ticks for motors.
    */
  uint32_t getMaxMotorTicks() const
  {
    return m_maxMotorTimerTicks;
  }

  /**
    * @brief  Prevent PID being clipped due to extremes of motor control. 
    * @brief  If PID goes beyond max motor ticks then subtract the overshoot from all motors.
    * @brief  If PID goes below min motor ticks then add the undershoot to all motors.
    * @param  motor1 to control
    * @param  motor2 to control
    * @param  motor3 to control
    * @param  motor4 to control
    */
  void multicopterMotorMagic(uint32_t * const motor1, uint32_t * const motor2, uint32_t * const motor3, uint32_t * const motor4)
  {
    uint32_t underShoot = 0;
    uint32_t overShoot = 0;
    //When PID undershoots the min motor timer tick, take the maximum undershoot value and add it to all motors. This allows full PID control at min throttle ...my version of air mode :-P
    if (*motor1 < m_minThrottle)
    {
      underShoot = m_minThrottle - *motor1;
    }
    else
    {
      if (*motor1 > m_maxMotorTimerTicks)
      {
        overShoot = *motor1 - m_maxMotorTimerTicks;
      }
    }

    if (*motor2 < m_minThrottle)
    {
      const uint32_t tempUnderShoot = m_minThrottle - *motor2;
      underShoot = (tempUnderShoot > underShoot) ? tempUnderShoot : underShoot;
    }
    else
    {
      if (*motor2 > m_maxMotorTimerTicks)
      {
        const uint32_t tempOverShoot = *motor2 - m_maxMotorTimerTicks;
        overShoot = (tempOverShoot > overShoot) ? tempOverShoot : overShoot;
      }
    }

    if (*motor3 < m_minThrottle)
    {
      const uint32_t tempUnderShoot = m_minThrottle - *motor3;
      underShoot = (tempUnderShoot > underShoot) ? tempUnderShoot : underShoot;
    }
    else
    {
      if (*motor3 > m_maxMotorTimerTicks)
      {
        const uint32_t tempOverShoot = *motor3 - m_maxMotorTimerTicks;
        overShoot = (tempOverShoot > overShoot) ? tempOverShoot : overShoot;
      }
    }

    if (*motor4 < m_minThrottle)
    {
      const uint32_t tempUnderShoot = m_minThrottle - *motor4;
      underShoot = (tempUnderShoot > underShoot) ? tempUnderShoot : underShoot;
    }
    else
    {
      if (*motor4 > m_maxMotorTimerTicks)
      {
        const uint32_t tempOverShoot = *motor4 - m_maxMotorTimerTicks;
        overShoot = (tempOverShoot > overShoot) ? tempOverShoot : overShoot;
      }
    }

    //We assume overShoot and underShoot cannot occur at the same time
    //If one or more motors PID is undershooting then add the different to all to maintain control at low throttle
    if (0 < underShoot)
    {
      *motor1 += underShoot;
      *motor2 += underShoot;
      *motor3 += underShoot;
      *motor4 += underShoot;
    }
    else
    {
      //If one or more motors PID is overshooting then subtract the different to all to maintain control at high throttle
      if (0 < overShoot)
      {
        *motor1 -= overShoot;
        *motor2 -= overShoot;
        *motor3 -= overShoot;
        *motor4 -= overShoot;
      }
    }
  };


  /**
    * @brief  Prevent PID being clipped due to extremes of motor control. 
    * @brief  If PID goes beyond max motor ticks then subtract the overshoot from all motors.
    * @brief  If PID goes below min motor ticks then add the undershoot to all motors.
    * @param  motor1 to control
    * @param  motor2 to control
    */
  void multicopterMotorMagic(uint32_t * const motor1, uint32_t * const motor2)
  {
    uint32_t underShoot = 0;
    uint32_t overShoot = 0;
    //When PID undershoots the min motor timer ticks, take the maximum undershoot value and add it to all motors. This allows full PID control at min throttle ...my version of air mode :-P
    //When PID overshoots the max motor timer ticks, take the maximum overshoot value and subtract it from all motors. This allows full PID control at max throttle ...my version of anti gravity mode :-P
    if (*motor1 < m_minThrottle)
    {
      underShoot = m_minThrottle - *motor1;
    }
    else
    {
      if (*motor1 > m_maxMotorTimerTicks)
      {
        overShoot = *motor1 - m_maxMotorTimerTicks;
      }
    }

    if (*motor2 < m_minThrottle)
    {
      const uint32_t tempUnderShoot = m_minThrottle - *motor2;
      underShoot = (tempUnderShoot > underShoot) ? tempUnderShoot : underShoot;
    }
    else
    {
      if (*motor2 > m_maxMotorTimerTicks)
      {
        const uint32_t tempOverShoot = *motor2 - m_maxMotorTimerTicks;
        overShoot = (tempOverShoot > overShoot) ? tempOverShoot : overShoot;
      }
    }

    //If one or more motors PID is undershooting then add the different to all to maintain control at low throttle
    if (0 < underShoot)
    {
      *motor1 += underShoot;
      *motor2 += underShoot;
    }
    else
    {
      if (0 < overShoot)
      {
        //If one or more motors PID is overshooting then subtract the different to all to maintain control at high throttle
        *motor1 -= overShoot;
        *motor2 -= overShoot;
      }
    }
  };
};


/**
* @brief    Plane with wing ailerons/flaps, rudder and elevator. 
* @note     Channel output map: Left aileron, right aileron elevator, rudder, motor 1, motor 2
* @note     3 position flaps work with this model.
*/
class PlaneFullHouse : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 4U;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t SERVO_3_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t SERVO_4_PIN    = BoardConfig::OUTPUT_PIN_4;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_5;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_6;

  //Configure this model as a quadcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  PlaneFullHouse() : ModelBase(m_modelConfig){};
  ~PlaneFullHouse(){};

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    int32_t negativeFlap = 0;

    if constexpr(Config::USE_FLAPS)
    {
      //Loss of precision in modifying flaps, but as its flaps we do not care.
      negativeFlap = map32(demands->flaps, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, RxBase::MID_NORMALISED, RxBase::MAX_NORMALISED);
    }

    // constraint add to prevent overflow
    const int32_t rollMinusFlap = constrain(demands->roll - negativeFlap, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t rollPlusFlap = constrain(demands->roll + negativeFlap, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t leftAileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollMinusFlap) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightAileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollPlusFlap) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t pitchTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->pitch) + (trim->servo3 * getTrimMultiplier()));
    const uint32_t yawTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->yaw) + (trim->servo4 * getTrimMultiplier()));
    writeServos(leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks);
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    int32_t negativeFlap = 0;

    if constexpr(Config::USE_FLAPS)
    {
      //Loss of precision in modifying flaps, but as its flaps we do not care.
      negativeFlap = map32(demands->flaps, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, 0, PIDF::PIDF_MAX_LIMIT);
    }

    // constraint add to prevent overflow
    const int32_t rollMinusFlap = constrain(demands->roll - negativeFlap, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t rollPlusFlap = constrain(demands->roll + negativeFlap, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t leftAileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollMinusFlap) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightAileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollPlusFlap) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t pitchTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->pitch) + (trim->servo3 * getTrimMultiplier()));
    const uint32_t yawTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->yaw) + (trim->servo4 * getTrimMultiplier()));
    writeServos(leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks);
  }

  /**
    * @brief  Motor mixer when not in rate mode
    * @param  demands - The rc demands
    */
  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t throttlePlusYaw = constrain(demands->throttle + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const int32_t throttleMinusYaw = constrain(demands->throttle - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const uint32_t motor1 = mapNormalisedMotorToTimerTicks(throttlePlusYaw);
      const uint32_t motor2 = mapNormalisedMotorToTimerTicks(throttleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }

  /**
    * @brief  Motor mixer when in rate/level mode
    * @param  demands - The rc demands
    */
  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Upscale throttle
      const int32_t modifiedThrottle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t modThrottlePlusYaw = constrain(modifiedThrottle + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const int32_t modThrottleMinusYaw = constrain(modifiedThrottle - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const uint32_t motor1 = mapRateMotorToTimerTicks(modThrottlePlusYaw);
      const uint32_t motor2 = mapRateMotorToTimerTicks(modThrottleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }
};


/**
* @brief    Plane with wing ailerons/flaps, with rudder and elevator mixed for V-tail i.e.'Talon' type model. 
* @note     Channel output map: left aileron, right aileron, Left taileron, right taileron, motor 1, motor 2
* @note     3 position flaps work with this model.
*/
class PlaneFullHouseVTail : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 4U;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t SERVO_3_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t SERVO_4_PIN    = BoardConfig::OUTPUT_PIN_4;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_5;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_6;

  //Configure this model...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  PlaneFullHouseVTail() : ModelBase(m_modelConfig){};
  ~PlaneFullHouseVTail(){};

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    int32_t negativeFlap = 0;

    if constexpr(Config::USE_FLAPS)
    {
      //Loss of precision in modifying flaps, but as its flaps we do not care.
      negativeFlap = map32(demands->flaps, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, RxBase::MID_NORMALISED, RxBase::MAX_NORMALISED);
    }
    // constraint add to prevent overflow
    const int32_t rollMinusFlap = constrain(demands->roll - negativeFlap, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t rollPlusFlap = constrain(demands->roll + negativeFlap, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t leftAileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollMinusFlap) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightAileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollPlusFlap) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t pitchTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->pitch) + (trim->servo3 * getTrimMultiplier()));
    const uint32_t yawTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->yaw) + (trim->servo4 * getTrimMultiplier()));
    writeServos(leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks);
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    int32_t negativeFlap = 0;

    if constexpr(Config::USE_FLAPS)
    {
      //Loss of precision in modifying flaps, but as its flaps we do not care.
      negativeFlap = map32(demands->flaps, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, 0, PIDF::PIDF_MAX_LIMIT);
    }
    // constraint add to prevent overflow
    const int32_t rollMinusFlap = constrain(demands->roll - negativeFlap, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t rollPlusFlap = constrain(demands->roll + negativeFlap, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t yawMinusPitch = constrain(demands->yaw - demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t yawPlusPitch = constrain(demands->yaw + demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t leftAileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollMinusFlap) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightAileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollPlusFlap) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t pitchTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(yawMinusPitch) + (trim->servo3 * getTrimMultiplier()));
    const uint32_t yawTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(yawPlusPitch) + (trim->servo4 * getTrimMultiplier()));
    writeServos(leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks);
  }

  /**
    * @brief  Motor mixer when not in rate mode
    * @param  demands - The rc demands
    */
  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t throttlePlusYaw = constrain(demands->throttle + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const int32_t throttleMinusYaw = constrain(demands->throttle - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const uint32_t motor1 = mapNormalisedMotorToTimerTicks(throttlePlusYaw);
      const uint32_t motor2 = mapNormalisedMotorToTimerTicks(throttleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }

  /**
    * @brief  Motor mixer when in rate/level mode
    * @param  demands - The rc demands
    */
  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Upscale throttle
      const int32_t modifiedThrottle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t modThrottlePlusYaw = constrain(modifiedThrottle + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const int32_t modThrottleMinusYaw = constrain(modifiedThrottle - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const uint32_t motor1 = mapRateMotorToTimerTicks(modThrottlePlusYaw);
      const uint32_t motor2 = mapRateMotorToTimerTicks(modThrottleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }
};


/**
* @brief    Plane with rudder elevator only. 
* @note     Roll and yaw corrections are mapped to rudder control. This makes for better rudder/elevator aerobatics but harder to tune.
* @note     Channel output map: rudder, elevator, motor 1, motor 2
* @note     Both roll and yaw gyro corrections are mixeded to rudder, this makes for better aerobatics but is harder to tune.
*/
class PlaneAdvancedRudderElevator : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 2U;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  PlaneAdvancedRudderElevator() : ModelBase(m_modelConfig){};
  ~PlaneAdvancedRudderElevator(){};

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    // constraint add to prevent overflow
    const int32_t rollPlusYaw = constrain(demands->roll + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t rudderTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollPlusYaw) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t elevatorTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->pitch) + (trim->servo2 * getTrimMultiplier()));
    writeServos(rudderTicks, elevatorTicks);
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    // constraint add to prevent overflow
    const int32_t rollPlusYaw = constrain(demands->roll + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t rudderTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollPlusYaw) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t elevatorTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->pitch) + (trim->servo2 * getTrimMultiplier()));
    writeServos(rudderTicks, elevatorTicks);
  }

  /**
    * @brief  Motor mixer when not in rate mode
    * @param  demands - The rc demands
    */
  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t throttlePlusYaw = constrain(demands->throttle + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const int32_t throttleMinusYaw = constrain(demands->throttle - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const uint32_t motor1 = mapNormalisedMotorToTimerTicks(throttlePlusYaw);
      const uint32_t motor2 = mapNormalisedMotorToTimerTicks(throttleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }

  /**
    * @brief  Motor mixer when in rate/level mode
    * @param  demands - The rc demands
    */
  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Upscale throttle
      const int32_t modifiedThrottle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t modThrottlePlusYaw = constrain(modifiedThrottle + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const int32_t modThrottleMinusYaw = constrain(modifiedThrottle - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const uint32_t motor1 = mapRateMotorToTimerTicks(modThrottlePlusYaw);
      const uint32_t motor2 = mapRateMotorToTimerTicks(modThrottleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }
};


/**
* @brief    Plane with rudder elevator only. 
* @note     Roll correction are mapped to rudder control.
* @note     Channel output map: rudder, elevator, motor 1, motor 2
*/
class PlaneRudderElevator : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 2U;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  PlaneRudderElevator() : ModelBase(m_modelConfig){};
  ~PlaneRudderElevator(){};

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    const uint32_t rudderTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->roll) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t elevatorTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->pitch) + (trim->servo2 * getTrimMultiplier()));
    writeServos(rudderTicks, elevatorTicks);
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    const uint32_t rudderTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->roll) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t elevatorTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->pitch) + (trim->servo2 * getTrimMultiplier()));
    writeServos(rudderTicks, elevatorTicks);
  }

  /**
    * @brief  Motor mixer when not in rate mode
    * @param  demands - The rc demands
    */
  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t throttlePlusYaw = constrain(demands->throttle + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const int32_t throttleMinusYaw = constrain(demands->throttle - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const uint32_t motor1 = mapNormalisedMotorToTimerTicks(throttlePlusYaw);
      const uint32_t motor2 = mapNormalisedMotorToTimerTicks(throttleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }

  /**
    * @brief  Motor mixer when in rate/level mode
    * @param  demands - The rc demands
    */
  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Upscale throttle
      const int32_t modifiedThrottle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t modThrottlePlusYaw = constrain(modifiedThrottle + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const int32_t modThrottleMinusYaw = constrain(modifiedThrottle - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const uint32_t motor1 = mapRateMotorToTimerTicks(modThrottlePlusYaw);
      const uint32_t motor2 = mapRateMotorToTimerTicks(modThrottleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }
};


/**
* @brief    Plane with V-tail controls only. 
* @note     Channel output map: left taileron, right taileron, motor 1, motor 2
*/
class PlaneVTail : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 2U;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  PlaneVTail() : ModelBase(m_modelConfig){};
  ~PlaneVTail(){};

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    // constraint add to prevent overflow
    const int32_t rollPlusPitch = constrain(demands->roll + demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t rollMinusPitch = constrain(demands->roll - demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t leftTaileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollPlusPitch) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightTaileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollMinusPitch) + (trim->servo2 * getTrimMultiplier()));
    writeServos(leftTaileronTicks, rightTaileronTicks);
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    // constraint add to prevent overflow
    const int32_t rollPlusPitch = constrain(demands->roll + demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t rollMinusPitch = constrain(demands->roll - demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t leftElevonTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollPlusPitch) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightElevonTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollMinusPitch) + (trim->servo2 * getTrimMultiplier()));
    writeServos(leftElevonTicks, rightElevonTicks);
  }

  /**
    * @brief  Motor mixer when not in rate mode
    * @param  demands - The rc demands
    */
  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t throttlePlusYaw = constrain(demands->throttle + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const int32_t throttleMinusYaw = constrain(demands->throttle - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const uint32_t motor1 = mapNormalisedMotorToTimerTicks(throttlePlusYaw);
      const uint32_t motor2 = mapNormalisedMotorToTimerTicks(throttleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }

  /**
    * @brief  Motor mixer when in rate/level mode
    * @param  demands - The rc demands
    */
  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Upscale throttle
      const int32_t modifiedThrottle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t modThrottlePlusYaw = constrain(modifiedThrottle + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const int32_t modThrottleMinusYaw = constrain(modifiedThrottle - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const uint32_t motor1 = mapRateMotorToTimerTicks(modThrottlePlusYaw);
      const uint32_t motor2 = mapRateMotorToTimerTicks(modThrottleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }
};


/**
* @brief    Flying wing class with 2 elevons and rudder if required. 
* @note     Channel output map: left elevon, right elevon, rudder, LEDc unused, motor 1, motor 2
*/
class PlaneFlyingWing : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 4U;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t SERVO_3_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t SERVO_4_PIN    = BoardConfig::OUTPUT_PIN_4;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_5;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_6;

  //Configure this model as a quadcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  PlaneFlyingWing() : ModelBase(m_modelConfig){};
  ~PlaneFlyingWing(){};

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    // constraint add to prevent overflow
    const int32_t rollPlusPitch = constrain(demands->roll + demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t rollMinusPitch = constrain(demands->roll - demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t leftElevonTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollPlusPitch) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightElevonTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollMinusPitch) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t rudderTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(demands->yaw) + (trim->servo3 * getTrimMultiplier()));
    writeServos(leftElevonTicks, rightElevonTicks, rudderTicks, getDefaultServoTicks(Actuator::CHANNEL_4));
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    // constraint add to prevent overflow
    const int32_t rollPlusPitch = constrain(demands->roll + demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t rollMinusPitch = constrain(demands->roll - demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t leftElevonTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollPlusPitch) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightElevonTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollMinusPitch) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t rudderTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->yaw) + (trim->servo3 * getTrimMultiplier()));
    writeServos(leftElevonTicks, rightElevonTicks, rudderTicks, getDefaultServoTicks(Actuator::CHANNEL_4));
  }

  /**
    * @brief  Motor mixer when not in rate mode
    * @param  demands - The rc demands
    */
  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t throttlePlusYaw = constrain(demands->throttle + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const int32_t throttleMinusYaw = constrain(demands->throttle - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
      const uint32_t motor1 = mapNormalisedMotorToTimerTicks(throttlePlusYaw);
      const uint32_t motor2 = mapNormalisedMotorToTimerTicks(throttleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }

  /**
    * @brief  Motor mixer when in rate/level mode
    * @param  demands - The rc demands
    */
  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    if constexpr(Config::USE_DIFFERENTIAL_THRUST)
    {
      //Upscale throttle
      const int32_t modifiedThrottle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      //Convert demands to timer ticks - constraint add to prevent overflow
      const int32_t modThrottlePlusYaw = constrain(modifiedThrottle + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const int32_t modThrottleMinusYaw = constrain(modifiedThrottle - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
      const uint32_t motor1 = mapRateMotorToTimerTicks(modThrottlePlusYaw);
      const uint32_t motor2 = mapRateMotorToTimerTicks(modThrottleMinusYaw);
      writeMotors(motor1, motor2);
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors(motor, motor);
    }
  }
};



/*Multicopters*/


/**
* @brief    Quadcopter X class. 
* @note     Quad x motor mapping:
* @note     4   2
* @note       x
* @note     3   1
* @note     Channel output map: motor 1, motor 2, motor 3, motor 4
*/
class QuadXCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 4U;
  static constexpr uint8_t NUMBER_SERVOS  = 0U;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t MOTOR_3_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t MOTOR_4_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model as a quadcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN},
          {0U, 0U, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  QuadXCopter() : ModelBase(m_modelConfig)
  {
    const uint32_t minThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_minThrottle = mapRateMotorToTimerTicks(minThrottle);
  };

  ~QuadXCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors(getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2), getDefaultMotorTicks(Actuator::CHANNEL_3), getDefaultMotorTicks(Actuator::CHANNEL_4));
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Rate mode only when armed
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle - demands->pitch + demands->roll - demands->yaw);
    uint32_t motor2 = mapRateMotorToTimerTicks(throttle + demands->pitch + demands->roll + demands->yaw);
    uint32_t motor3 = mapRateMotorToTimerTicks(throttle - demands->pitch - demands->roll + demands->yaw);
    uint32_t motor4 = mapRateMotorToTimerTicks(throttle + demands->pitch - demands->roll - demands->yaw);

    multicopterMotorMagic(&motor1, &motor2, &motor3, &motor4);

    motor1 = constrain(motor1, m_minThrottle, getMaxMotorTicks());
    motor2 = constrain(motor2, m_minThrottle, getMaxMotorTicks());
    motor3 = constrain(motor3, m_minThrottle, getMaxMotorTicks());
    motor4 = constrain(motor4, m_minThrottle, getMaxMotorTicks());

    writeMotors(motor1, motor2, motor3, motor4);
  };
};


/**
* @brief    Quadcopter Plus class. 
* @note     Quad + motor mapping:
* @note       1
* @note     4 + 2
* @note       3
* @note     Channel output map: motor 1, motor 2, motor 3, motor 4
*/
class QuadPlusCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 4U;
  static constexpr uint8_t NUMBER_SERVOS  = 0U;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t MOTOR_3_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t MOTOR_4_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model as a quadcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN},
          {0U, 0U, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  QuadPlusCopter() : ModelBase(m_modelConfig)
  {
    const uint32_t minThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_minThrottle = mapRateMotorToTimerTicks(minThrottle);
  };

  ~QuadPlusCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors(getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2), getDefaultMotorTicks(Actuator::CHANNEL_3), getDefaultMotorTicks(Actuator::CHANNEL_4));
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Rate mode only when armed
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle - demands->pitch - demands->yaw);
    uint32_t motor2 = mapRateMotorToTimerTicks(throttle - demands->roll + demands->yaw);
    uint32_t motor3 = mapRateMotorToTimerTicks(throttle + demands->pitch - demands->yaw);
    uint32_t motor4 = mapRateMotorToTimerTicks(throttle + demands->roll + demands->yaw);

    multicopterMotorMagic(&motor1, &motor2, &motor3, &motor4);

    motor1 = constrain(motor1, m_minThrottle, getMaxMotorTicks());
    motor2 = constrain(motor2, m_minThrottle, getMaxMotorTicks());
    motor3 = constrain(motor3, m_minThrottle, getMaxMotorTicks());
    motor4 = constrain(motor4, m_minThrottle, getMaxMotorTicks());

    writeMotors(motor1, motor2, motor3, motor4);
  };
};


/**
* @brief    Chinook class. 
* @note     Channel output map: Front servo, rear servo, motor 1, motor 2
*/
class ChinookCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 2U;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model as a quadcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  ChinookCopter() : ModelBase(m_modelConfig)
  {
    const uint32_t minThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_minThrottle = mapRateMotorToTimerTicks(minThrottle);
  };

  ~ChinookCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors(getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2));
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Rate mode only when armed
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle - demands->pitch);
    uint32_t motor2 = mapRateMotorToTimerTicks(throttle + demands->pitch);

    multicopterMotorMagic(&motor1, &motor2);

    motor1 = constrain(motor1, m_minThrottle, getMaxMotorTicks());
    motor2 = constrain(motor2, m_minThrottle, getMaxMotorTicks());

    writeMotors(motor1, motor2);
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed - constraint add to prevent overflow
    const int32_t rollPlusYaw = constrain(demands->roll  + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t rollMinusYaw = constrain(demands->roll  - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(rollPlusYaw) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapNormalisedServoToTimerTicks(rollMinusYaw) + (trim->servo2 * getTrimMultiplier());
    writeServos(servo1, servo2);
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed - constraint add to prevent overflow
    const int32_t rollPlusYaw = constrain(demands->roll  + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t rollMinusYaw = constrain(demands->roll  - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t servo1 = mapRateServoToTimerTicks(rollPlusYaw) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapRateServoToTimerTicks(rollMinusYaw) + (trim->servo2 * getTrimMultiplier());
    writeServos(servo1, servo2);
  }
};


/**
* @brief    Bicopter class. 
* @note     Channel output left servo: right servo, motor 1, motor 2
*/
class BiCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 2U;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model as a bicopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  BiCopter() : ModelBase(m_modelConfig)
  {
    const uint32_t minThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_minThrottle = mapRateMotorToTimerTicks(minThrottle);
  };

  ~BiCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors(getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2));
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle + demands->roll);
    uint32_t motor2 = mapRateMotorToTimerTicks(throttle - demands->roll);

    multicopterMotorMagic(&motor1, &motor2);

    motor1 = constrain(motor1, m_minThrottle, getMaxMotorTicks());
    motor2 = constrain(motor2, m_minThrottle, getMaxMotorTicks());

    writeMotors(motor1, motor2);
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed - constraint add to prevent overflow
    const int32_t yawPlusPitch = constrain(demands->yaw  + demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t yawMinusPitch = constrain(demands->yaw  - demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(yawPlusPitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapNormalisedServoToTimerTicks(yawMinusPitch) + (trim->servo2 * getTrimMultiplier());
    writeServos(servo1, servo2);
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed - constraint add to prevent overflow
    const int32_t yawPlusPitch = constrain(demands->yaw  + demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t yawMinusPitch = constrain(demands->yaw  - demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t servo1 = mapRateServoToTimerTicks(yawPlusPitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapRateServoToTimerTicks(yawMinusPitch) + (trim->servo2 * getTrimMultiplier());
    writeServos(servo1, servo2);
  }
};


/**
* @brief    Tricopter class. 
* @note     Channel output map: tail servo, LEDc unused, motor 1, motor 2, motor 3
*/
class TriCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 4U; //Puposely defining 4 as LEDc channels are pairs. We need 2 motor pairs then a lower refresh rate servo.
  static constexpr uint8_t NUMBER_SERVOS  = 2U; //Purposely defining 2 as LEDc channles are pairs.
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t MOTOR_3_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t MOTOR_4_PIN    = BoardConfig::OUTPUT_PIN_4;  //Puposely defining 4 as LEDc channels are pairs. We need 2 motor pairs then a lower refresh rate servo.
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_5;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_6;

  //Configure this model as a tricopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN},
          {SERVO_1_PIN, SERVO_2_PIN, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  TriCopter() : ModelBase(m_modelConfig)
  {
    const uint32_t minThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_minThrottle = mapRateMotorToTimerTicks(minThrottle);
  };

  ~TriCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors(getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2), getDefaultMotorTicks(Actuator::CHANNEL_3), getDefaultMotorTicks(Actuator::CHANNEL_4));
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    const int32_t oneThirdPitch = static_cast<int32_t>((static_cast<float>(demands->roll) * 1.3333f) + 0.5f);
    const int32_t twoThirdPitch = static_cast<int32_t>((static_cast<float>(demands->roll) * 0.6666f) + 0.5f);
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle + oneThirdPitch);                   //Rear motor
    uint32_t motor2 = mapRateMotorToTimerTicks(throttle + demands->roll - twoThirdPitch);   //Left motor
    uint32_t motor3 = mapRateMotorToTimerTicks(throttle - demands->roll - twoThirdPitch);   //Right motor
    uint32_t motor4 = getDefaultMotorTicks(Actuator::CHANNEL_4);

    multicopterMotorMagic(&motor1, &motor2, &motor3, &motor4);

    motor1 = constrain(motor1, m_minThrottle, getMaxMotorTicks());
    motor2 = constrain(motor2, m_minThrottle, getMaxMotorTicks());
    motor3 = constrain(motor3, m_minThrottle, getMaxMotorTicks());
    motor4 = constrain(motor4, m_minThrottle, getMaxMotorTicks());

    writeMotors(motor1, motor2, motor3, motor4);
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(demands->yaw) + (trim->servo1 * getTrimMultiplier());
    writeServos(servo1, getDefaultServoTicks(Actuator::CHANNEL_2));
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed
    const uint32_t servo1 = mapRateServoToTimerTicks(demands->yaw) + (trim->servo1 * getTrimMultiplier());
    writeServos(servo1, getDefaultServoTicks(Actuator::CHANNEL_2));
  }
};



/**
* @brief    Dualcopter class. 
* @note     Channel output map: roll servo, pitch servo, motor 1, motor 2
*/
class DualCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 2U;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_2;
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_4;

  //Configure this model as a dualcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, 0U, 0U},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  DualCopter() : ModelBase(m_modelConfig)
  {
    const uint32_t minThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_minThrottle = mapRateMotorToTimerTicks(minThrottle);
  };

  ~DualCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors(getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2));
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle - demands->yaw);
    uint32_t motor2 = mapRateMotorToTimerTicks(throttle + demands->yaw);

    multicopterMotorMagic(&motor1, &motor2);

    motor1 = constrain(motor1, m_minThrottle, getMaxMotorTicks());
    motor2 = constrain(motor2, m_minThrottle, getMaxMotorTicks());

    writeMotors(motor1, motor2);
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(demands->pitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapNormalisedServoToTimerTicks(demands->roll) + (trim->servo2 * getTrimMultiplier());
    writeServos(servo1, servo2);
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed
    const uint32_t servo1 = mapRateServoToTimerTicks(demands->pitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapRateServoToTimerTicks(demands->roll) + (trim->servo2 * getTrimMultiplier());
    writeServos(servo1, servo2);
  }
};


/**
* @brief    Singlecopter class. 
* @note     4 servos arranged with 90 degree speration.
* @note     Channel output map: servo 1, servo 2, servo 3, servo 4, motor 1, LEDc unused
*/
class SingleCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U; //Puposely defining 2 as LEDc channels are pairs. We need 2 motor pairs then lower refresh rate servos.
  static constexpr uint8_t NUMBER_SERVOS  = 4U;
  static constexpr uint8_t MOTOR_1_PIN    = BoardConfig::OUTPUT_PIN_1;
  static constexpr uint8_t MOTOR_2_PIN    = BoardConfig::OUTPUT_PIN_2;  //Puposely defining 4 as LEDc channels are pairs. We need 2 motor pairs then a lower refresh rate servo.
  static constexpr uint8_t SERVO_1_PIN    = BoardConfig::OUTPUT_PIN_3;
  static constexpr uint8_t SERVO_2_PIN    = BoardConfig::OUTPUT_PIN_4;
  static constexpr uint8_t SERVO_3_PIN    = BoardConfig::OUTPUT_PIN_5;
  static constexpr uint8_t SERVO_4_PIN    = BoardConfig::OUTPUT_PIN_6;

  //Configure this model as a singlecopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, 0U, 0U},
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  SingleCopter() : ModelBase(m_modelConfig){};

  ~SingleCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors(getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2));
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle);
    writeMotors(motor1, getDefaultMotorTicks(Actuator::CHANNEL_2));
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed - constraint add to prevent overflow
    const int32_t rollPlusYaw   = constrain(demands->roll  + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t pitchPlusYaw  = constrain(demands->pitch + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(rollPlusYaw)  + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapNormalisedServoToTimerTicks(pitchPlusYaw) + (trim->servo2 * getTrimMultiplier());
    const uint32_t servo3 = mapNormalisedServoToTimerTicks(rollPlusYaw)  + (trim->servo3 * getTrimMultiplier());
    const uint32_t servo4 = mapNormalisedServoToTimerTicks(pitchPlusYaw) + (trim->servo4 * getTrimMultiplier());    
    writeServos(servo1, servo2, servo3, servo4);
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed - constraint add to prevent overflow
    const int32_t rollPlusYaw   = constrain(demands->roll  + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t pitchPlusYaw  = constrain(demands->pitch + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t servo1 = mapRateServoToTimerTicks(rollPlusYaw)  + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapRateServoToTimerTicks(pitchPlusYaw) + (trim->servo2 * getTrimMultiplier());
    const uint32_t servo3 = mapRateServoToTimerTicks(rollPlusYaw)  + (trim->servo3 * getTrimMultiplier());
    const uint32_t servo4 = mapRateServoToTimerTicks(pitchPlusYaw) + (trim->servo4 * getTrimMultiplier());    
    writeServos(servo1, servo2, servo3, servo4);
  }
};