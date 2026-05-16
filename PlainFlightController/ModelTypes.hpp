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
#include "DemandProcessor.hpp"

static constexpr uint8_t PIN_UNUSED = 0xFF;  // Marker for unused outputs

/**
* @brief    Base class for all model types.
* @note     Arbitrary numbers of servos and motors up to LedcServo::MAX_LEDC_CHANNELS can be made
* @note     A maximum of 4 different refresh rates are possible but we only use two
*/
class ModelBase : public Utilities
{
public:
  //Structure used to define model type and actuators required.
  struct ModelConfig
  {
    // Servos and Motors are assigned to contiguous positions in this array
    // Servos occupy [0:numberServos-1] and Motors occupy [numberServos: numberServos+numberMotors-1]
    uint8_t outputPins[LedcServo::MAX_LEDC_CHANNELS];  // MAX_LEDC_CHANNELS = 8 for the ESP32-S3
    LedcServo::RefreshRate motorRefresh;
    LedcServo::RefreshRate servoRefresh;
    uint8_t numberMotors;
    uint8_t numberServos;
  };

  ModelBase(ModelConfig modelConfig) 
    : m_modelConfig(modelConfig),
      m_idleUp(IDLE_UP),
      m_minThrottle(MIN_THROTTLE),
      m_totalOutputs(modelConfig.numberServos + modelConfig.numberMotors)
  {
    // Ensure not too many outputs are declared
    assert(m_totalOutputs <= LedcServo::MAX_LEDC_CHANNELS);
    // limit on timer numbers is satisfied as we admit only two refresh frequencies

    for (uint8_t i = 0U; i < m_totalOutputs; i++)
    {
        bool isServo = i < m_modelConfig.numberServos;
        outputs[i] = LedcServo(
            m_modelConfig.outputPins[i],
            isServo ? m_modelConfig.servoRefresh : m_modelConfig.motorRefresh,
            isServo ? LedcServo::MID_MICRO_SECONDS : LedcServo::MIN_MICRO_SECONDS,
            Config::REVERSE_OUTPUT[i],
            Config::EXTEND_SERVO_TRAVEL_RANGE
        );
    }
    // Max/Min Timer ticks are calulated during construction of LedcServo objects
    m_minServoTimerTicks = static_cast<int32_t>(servoAt(0).getMinTimerTicks());//All servos will be the same refresh rate
    m_maxServoTimerTicks = static_cast<int32_t>(servoAt(0).getMaxTimerTicks());
    m_minMotorTimerTicks = static_cast<int32_t>(motorAt(0).getMinTimerTicks());//And/or all motors will be the same refresh rate
    m_maxMotorTimerTicks = static_cast<int32_t>(motorAt(0).getMaxTimerTicks());

    const int32_t rateMinThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED,
                                      -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_rateMinThrottleTicks = static_cast<uint32_t>(map32(rateMinThrottle, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT,
                                                      m_minMotorTimerTicks, m_maxMotorTimerTicks));
  }

  ~ModelBase(){};

  /**
    * @brief  Initialises servos and motors
    */
  virtual void begin()
  {
    for (uint8_t i=0U; i<m_totalOutputs; i++)
    {
      outputs[i].begin();
    }

    if constexpr(Config::CALIBRATE_ESC)
    {
      for (uint8_t i=0U; i<m_modelConfig.numberMotors; i++)
      {
        motorAt(i).setTimerTicks(motorAt(i).getMaxTimerTicks());
      }

      delay(LedcServo::CALIBRATE_ESC_DELAY);

      for (uint8_t i=0U; i<m_modelConfig.numberMotors; i++)
      {
        motorAt(i).setTimerTicks(motorAt(i).getMinTimerTicks());
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
  int32_t getTrimMultiplier() const {return servoAt(0).getTrimMultiplier();}

private:
  //Variables
  ModelConfig m_modelConfig;
  int32_t m_minServoTimerTicks;
  int32_t m_maxServoTimerTicks;
  int32_t m_minMotorTimerTicks;
  int32_t m_maxMotorTimerTicks;
  uint32_t m_rateMinThrottleTicks;

  //Objects
  LedcServo outputs[LedcServo::MAX_LEDC_CHANNELS];
  LedcServo& servoAt(uint8_t i) { return outputs[i]; }
  const LedcServo& servoAt(uint8_t i) const { return outputs[i]; }
  LedcServo& motorAt(uint8_t i) { return outputs[m_modelConfig.numberServos + i]; }
  const LedcServo& motorAt(uint8_t i) const { return outputs[m_modelConfig.numberServos + i]; }

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
  const int32_t m_idleUp;
  const int32_t m_minThrottle;
  const uint8_t m_totalOutputs;

  /**
   * @brief  Internal helper to write values to a subset of outputs.
   * @param  startIndex The starting index in the 'outputs' array.
   * @param  values     The list of timer ticks to apply.
   * @param  label      String label for debug output ("Motor" or "Servo").
   */
  void writeToOutputs(uint8_t startIndex, std::initializer_list<uint32_t> values, const char* label)
  {
      static uint64_t debugUpdateTime = 0U;
      uint8_t i = 0U;
      for (uint32_t v : values)
      {
          outputs[startIndex + i++].setTimerTicks(v);
      }

      if constexpr (Config::DEBUG_OUTPUT)
      {
          const uint64_t nowTime = millis();
          if (debugUpdateTime <= nowTime)
          {
              uint8_t j = 0U;
              for (uint32_t v : values) 
              {
                  Serial.printf("%s %d: %u\n", label, j++, v);
              }
              debugUpdateTime = nowTime + 100U;
          }
      }
  }

  /**
   * @brief  Writes values to motors.
   */
  void writeMotors(std::initializer_list<uint32_t> values)
  {
      writeToOutputs(m_modelConfig.numberServos, values, "Motor");
  }

  /**
   * @brief  Writes values to servos.
   */
  void writeServos(std::initializer_list<uint32_t> values)
  {
      writeToOutputs(0U, values, "Servo");
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
    return motorAt(static_cast<uint32_t>(number)).getDefaultTimerTicks();
  }

  /**
    * @brief  Gets the default timer ticks for servos i.e. servo centred.
    * @param  
    */
  uint32_t getDefaultServoTicks(Actuator number)
  {
    return servoAt(static_cast<uint32_t>(number)).getDefaultTimerTicks();
  }

  /**
    * @brief  Gets the max timer ticks for motors.
    */
  uint32_t getMaxMotorTicks() const
  {
    return m_maxMotorTimerTicks;
  }

  /**
    * @brief  Gets the min "Rate" timer ticks for motors.
    */
  uint32_t getRateMinThrottleTicks() const
  {
    return m_rateMinThrottleTicks;
  }

  /**
    * @brief Prevent PID being clipped due to extremes of motor control.
    * @brief If any PID value goes beyond max motor ticks then subtract the MAX overshoot (across all motors) from EVERY motor.
    * @brief If any PID value goes below min motor ticks then add the MAX undershoot (across all motors) to EVERY motor.
    * @brief This preserves full PID authority at both min and max throttle (your "air mode" / "anti-gravity mode").
    *
    * @param motors     Array of pointers to the motor timer values (one pointer per motor).
    *
    * Call example for a bicopter (2 motors):
    *     multicopterMotorMagic({motor1, motor2});
    *
    * Call example for a quadcopter (4 motors):
    *     multicopterMotorMagic({motor1, motor2, motor3, motor4});
    *
    * Call example for a hexacopter (6 motors) or octocopter (8 motors) follows the same pattern.
  */


  void multicopterMotorMagic(std::initializer_list<uint32_t*> motors)
  {
      uint32_t underShoot = 0;
      uint32_t overShoot  = 0;

      // Find the SINGLE largest undershoot OR overshoot across ALL motors.
      // We only check overshoot on a motor if it is NOT undershooting (exactly as the original code did).
      for (uint32_t* motor : motors)
      {
          const uint32_t val = *motor;

          if (val < m_minThrottle)
          {
              const uint32_t tempUnder = m_minThrottle - val;
              if (tempUnder > underShoot)
                  underShoot = tempUnder;
          }
          else if (val > m_maxMotorTimerTicks)
          {
              const uint32_t tempOver = val - m_maxMotorTimerTicks;
              if (tempOver > overShoot)
                  overShoot = tempOver;
          }
      }

      // We assume undershoot and overshoot cannot occur at the same time.
      // Apply the correction to EVERY motor so the whole set is shifted together.
      if (0 < underShoot)
      {
          // Add the max undershoot to all motors → full PID control remains at minimum throttle.
          for (uint32_t* motor : motors)
            *motor += underShoot;
      }
      else if (0 < overShoot)
      {
          // Subtract the max overshoot from all motors → full PID control remains at maximum throttle.
          for (uint32_t* motor : motors)
            *motor -= overShoot;
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
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t SERVO_3_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t SERVO_4_PIN    = Config::ESP32S3.OUTPUT_4;
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_5;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_6;

  //Configure this model as PlaneFullHouse ...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED},
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
    int32_t negativeFlap = 0U;

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
    writeServos({leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks});
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    int32_t negativeFlap = 0U;

    if constexpr(Config::USE_FLAPS)
    {
      //Loss of precision in modifying flaps, but as its flaps we do not care.
      negativeFlap = map32(demands->flaps, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, 0U, PIDF::PIDF_MAX_LIMIT);
    }

    // constraint add to prevent overflow
    const int32_t rollMinusFlap = constrain(demands->roll - negativeFlap, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t rollPlusFlap = constrain(demands->roll + negativeFlap, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t leftAileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollMinusFlap) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightAileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollPlusFlap) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t pitchTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->pitch) + (trim->servo3 * getTrimMultiplier()));
    const uint32_t yawTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(demands->yaw) + (trim->servo4 * getTrimMultiplier()));
    writeServos({leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t SERVO_3_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t SERVO_4_PIN    = Config::ESP32S3.OUTPUT_4;
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_5;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_6;

  //Configure this model as PlaneFullHouseVTail...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED},
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
    int32_t negativeFlap = 0U;

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
    writeServos({leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks});
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The rc demands.
    * @param  trim - The servo trims to apply
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    int32_t negativeFlap = 0U;

    if constexpr(Config::USE_FLAPS)
    {
      //Loss of precision in modifying flaps, but as its flaps we do not care.
      negativeFlap = map32(demands->flaps, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, 0U, PIDF::PIDF_MAX_LIMIT);
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
    writeServos({leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as PlaneAdvancedRudderElevator...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
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
    writeServos({rudderTicks, elevatorTicks});
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
    writeServos({rudderTicks, elevatorTicks});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as PlaneRudderElevator...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
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
    writeServos({rudderTicks, elevatorTicks});
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
    writeServos({rudderTicks, elevatorTicks});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as PlaneVTail...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
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
    writeServos({leftTaileronTicks, rightTaileronTicks});
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
    writeServos({leftElevonTicks, rightElevonTicks});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t SERVO_3_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t SERVO_4_PIN    = Config::ESP32S3.OUTPUT_4;
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_5;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_6;

  //Configure this model as flying wing...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED},
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
    writeServos({leftElevonTicks, rightElevonTicks, rudderTicks, getDefaultServoTicks(Actuator::CHANNEL_4)});
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
    writeServos({leftElevonTicks, rightElevonTicks, rudderTicks, getDefaultServoTicks(Actuator::CHANNEL_4)});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
      writeMotors({motor1, motor2});
    }
    else
    {
      const uint32_t motor = mapNormalisedMotorToTimerTicks(demands->throttle);
      writeMotors({motor, motor});
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
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t MOTOR_3_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t MOTOR_4_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as a quadcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  QuadXCopter() : ModelBase(m_modelConfig){};

  ~QuadXCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2), getDefaultMotorTicks(Actuator::CHANNEL_3), getDefaultMotorTicks(Actuator::CHANNEL_4)});
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

    multicopterMotorMagic({&motor1, &motor2, &motor3, &motor4});

    motor1 = constrain(motor1, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor2 = constrain(motor2, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor3 = constrain(motor3, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor4 = constrain(motor4, getRateMinThrottleTicks(), getMaxMotorTicks());

    writeMotors({motor1, motor2, motor3, motor4});
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
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t MOTOR_3_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t MOTOR_4_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as a quadcopter +...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  QuadPlusCopter() : ModelBase(m_modelConfig){};

  ~QuadPlusCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2), getDefaultMotorTicks(Actuator::CHANNEL_3), getDefaultMotorTicks(Actuator::CHANNEL_4)});
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

    multicopterMotorMagic({&motor1, &motor2, &motor3, &motor4});

    motor1 = constrain(motor1, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor2 = constrain(motor2, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor3 = constrain(motor3, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor4 = constrain(motor4, getRateMinThrottleTicks(), getMaxMotorTicks());

    writeMotors({motor1, motor2, motor3, motor4});
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
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as chinook...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  ChinookCopter() : ModelBase(m_modelConfig){};

  ~ChinookCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2)});
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

    multicopterMotorMagic({&motor1, &motor2});

    motor1 = constrain(motor1, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor2 = constrain(motor2, getRateMinThrottleTicks(), getMaxMotorTicks());

    writeMotors({motor1, motor2});
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed - constraint add to prevent overflow
    const int32_t rollPlusYaw = constrain(demands->roll  + demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t rollMinusYaw = constrain(demands->roll  - demands->yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(rollPlusYaw) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapNormalisedServoToTimerTicks(rollMinusYaw) + (trim->servo2 * getTrimMultiplier());
    writeServos({servo1, servo2});
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed - constraint add to prevent overflow
    const int32_t rollPlusYaw = constrain(demands->roll  + demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t rollMinusYaw = constrain(demands->roll  - demands->yaw, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t servo1 = mapRateServoToTimerTicks(rollPlusYaw) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapRateServoToTimerTicks(rollMinusYaw) + (trim->servo2 * getTrimMultiplier());
    writeServos({servo1, servo2});
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
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as a bicopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  BiCopter() : ModelBase(m_modelConfig){};

  ~BiCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2)});
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

    multicopterMotorMagic({&motor1, &motor2});

    motor1 = constrain(motor1, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor2 = constrain(motor2, getRateMinThrottleTicks(), getMaxMotorTicks());

    writeMotors({motor1, motor2});
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed - constraint add to prevent overflow
    const int32_t yawPlusPitch = constrain(demands->yaw  + demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t yawMinusPitch = constrain(demands->yaw  - demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(yawPlusPitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapNormalisedServoToTimerTicks(yawMinusPitch) + (trim->servo2 * getTrimMultiplier());
    writeServos({servo1, servo2});
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed - constraint add to prevent overflow
    const int32_t yawPlusPitch = constrain(demands->yaw  + demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const int32_t yawMinusPitch = constrain(demands->yaw  - demands->pitch, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    const uint32_t servo1 = mapRateServoToTimerTicks(yawPlusPitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapRateServoToTimerTicks(yawMinusPitch) + (trim->servo2 * getTrimMultiplier());
    writeServos({servo1, servo2});
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
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t MOTOR_3_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t MOTOR_4_PIN    = Config::ESP32S3.OUTPUT_4; //Puposely defining 4 as LEDc channels are pairs. We need 2 motor pairs then a lower refresh rate servo.
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_5;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_6;

  //Configure this model as a tricopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN, PIN_UNUSED, PIN_UNUSED},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  TriCopter() : ModelBase(m_modelConfig){};

  ~TriCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2), getDefaultMotorTicks(Actuator::CHANNEL_3), getDefaultMotorTicks(Actuator::CHANNEL_4)});
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

    multicopterMotorMagic({&motor1, &motor2, &motor3, &motor4});

    motor1 = constrain(motor1, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor2 = constrain(motor2, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor3 = constrain(motor3, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor4 = constrain(motor4, getRateMinThrottleTicks(), getMaxMotorTicks());
    writeMotors({motor1, motor2, motor3, motor4});
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(demands->yaw) + (trim->servo1 * getTrimMultiplier());
    writeServos({servo1, getDefaultServoTicks(Actuator::CHANNEL_2)});
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed
    const uint32_t servo1 = mapRateServoToTimerTicks(demands->yaw) + (trim->servo1 * getTrimMultiplier());
    writeServos({servo1, getDefaultServoTicks(Actuator::CHANNEL_2)});
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
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_4;

  //Configure this model as a dualcopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  DualCopter() : ModelBase(m_modelConfig){};

  ~DualCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2)});
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

    multicopterMotorMagic({&motor1, &motor2});

    motor1 = constrain(motor1, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor2 = constrain(motor2, getRateMinThrottleTicks(), getMaxMotorTicks());

    writeMotors({motor1, motor2});
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(demands->pitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapNormalisedServoToTimerTicks(demands->roll) + (trim->servo2 * getTrimMultiplier());
    writeServos({servo1, servo2});
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed
    const uint32_t servo1 = mapRateServoToTimerTicks(demands->pitch) + (trim->servo1 * getTrimMultiplier());
    const uint32_t servo2 = mapRateServoToTimerTicks(demands->roll) + (trim->servo2 * getTrimMultiplier());
    writeServos({servo1, servo2});
  }
};


/**
* @brief    Singlecopter class. 
* @note     4 servos arranged with 90 degree separation.
* @note     Channel output map: servo 1, servo 2, servo 3, servo 4, motor 1, LEDc unused
*/
class SingleCopter : public ModelBase
{
public:
  static constexpr uint8_t NUMBER_MOTORS  = 2U;
  static constexpr uint8_t NUMBER_SERVOS  = 4U;
  static constexpr uint8_t MOTOR_1_PIN    = Config::ESP32S3.OUTPUT_1;
  static constexpr uint8_t MOTOR_2_PIN    = Config::ESP32S3.OUTPUT_2;
  static constexpr uint8_t SERVO_1_PIN    = Config::ESP32S3.OUTPUT_3;
  static constexpr uint8_t SERVO_2_PIN    = Config::ESP32S3.OUTPUT_4;
  static constexpr uint8_t SERVO_3_PIN    = Config::ESP32S3.OUTPUT_5;
  static constexpr uint8_t SERVO_4_PIN    = Config::ESP32S3.OUTPUT_6;

  //Configure this model as a singlecopter...
  static constexpr ModelBase::ModelConfig m_modelConfig =
      {
          {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN, MOTOR_1_PIN, MOTOR_2_PIN, PIN_UNUSED, PIN_UNUSED},
          Config::MOTOR_REFRESH_RATE,
          Config::SERVO_REFRESH_RATE,
          NUMBER_MOTORS,
          NUMBER_SERVOS,
      };

  SingleCopter() : ModelBase(m_modelConfig){};

  ~SingleCopter(){};

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::CHANNEL_1), getDefaultMotorTicks(Actuator::CHANNEL_2)});
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle);
    writeMotors({motor1, getDefaultMotorTicks(Actuator::CHANNEL_2)});
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
    writeServos({servo1, servo2, servo3, servo4});
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
    writeServos({servo1, servo2, servo3, servo4});
  }
};