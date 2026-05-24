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
#include "InternalConfig.hpp"

/**
* @brief    Base class for all model types.
* @note     Arbitrary numbers of servos and motors up to LedcServo::MAX_LEDC_CHANNELS can be made
* @note     A maximum of 4 different refresh rates are possible but we only use two
*/
class ModelBase : public Utilities
{
public:
  static constexpr uint8_t PIN_UNUSED = 0xFFU;  // Marker for unused outputs
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

  ModelBase() 
    : m_idleUp(IDLE_UP),
      m_minThrottle(MIN_THROTTLE),
      m_totalOutputs(m_modelConfig.numberServos + m_modelConfig.numberMotors)
  {
    // Ensure not too many outputs are declared
    assert(m_totalOutputs <= LedcServo::MAX_LEDC_CHANNELS);
    // limit on timer numbers is satisfied as we admit only two refresh frequencies

    // rearrange channel reverse config values for looped instantiation
    static constexpr bool REVERSE_OUTPUT[LedcServo::MAX_LEDC_CHANNELS] = {
      Config::REVERSE_OUTPUT_1, Config::REVERSE_OUTPUT_2, Config::REVERSE_OUTPUT_3, Config::REVERSE_OUTPUT_4, 
      Config::REVERSE_OUTPUT_5, Config::REVERSE_OUTPUT_6, Config::REVERSE_OUTPUT_7, Config::REVERSE_OUTPUT_8
    };

    for (uint8_t i = 0U; i < m_totalOutputs; i++)
    {
        bool isServo = i < m_modelConfig.numberServos;
        outputs[i] = LedcServo(
            m_modelConfig.outputPins[i],
            isServo ? m_modelConfig.servoRefresh : m_modelConfig.motorRefresh,
            isServo ? LedcServo::MID_MICRO_SECONDS : LedcServo::MIN_MICRO_SECONDS,
            Config::EXTEND_SERVO_TRAVEL_RANGE,
            REVERSE_OUTPUT[i]
        );
    }
    // Max/Min Timer ticks are calulated during construction of LedcServo objects
    m_minServoTimerTicks = static_cast<int32_t>(servoAt(0).getMinTimerTicks());//All servos will be the same refresh rate
    m_maxServoTimerTicks = static_cast<int32_t>(servoAt(0).getMaxTimerTicks());
    m_minMotorTimerTicks = static_cast<int32_t>(motorAt(0).getMinTimerTicks());//And/or all motors will be the same refresh rate
    m_maxMotorTimerTicks = static_cast<int32_t>(motorAt(0).getMaxTimerTicks());

    const int32_t rateMinThrottle = map32(MIN_THROTTLE, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED,
                                      -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);
    m_rateMinThrottleTicks = mapRateMotorToTimerTicks(rateMinThrottle);
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

  /**
    * @brief  Servo mixer
    * @param  demands - The RC demands
    * @param  trim - The saved servo trim values
    * @note   Default implementation asserts if the model declares servos but does not override this method.
    */
  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim)
  {
    assert(m_modelConfig.numberServos == 0 && "servoMixer must be overridden in models that declare servos");
  }

  /**
    * @brief  Servo mixer
    * @param  demands - The RC demands
    * @param  trim - The saved servo trim values
    * @note   Default implementation asserts if the model declares servos but does not override this method.
    */
  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim)
  {
    assert(m_modelConfig.numberServos == 0 && "servoRateMixer not overridden for a model declared with servos");
  }

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
  // Helper function
  static constexpr ModelConfig makeModelConfig()
  {
      ModelConfig cfg = {};

      for (uint8_t i = 0; i < LedcServo::MAX_LEDC_CHANNELS; i++)
          cfg.outputPins[i] = PIN_UNUSED;

      for (uint8_t i = 0; i < InternalConfig::NUMBER_SERVOS; i++)
          cfg.outputPins[i] = Config::SERVO_PINS[i];

      for (uint8_t i = 0; i < InternalConfig::NUMBER_MOTORS; i++)
          cfg.outputPins[InternalConfig::NUMBER_SERVOS + i] = Config::MOTOR_PINS[i];

      cfg.numberServos = InternalConfig::NUMBER_SERVOS;
      cfg.numberMotors = InternalConfig::NUMBER_MOTORS;
      cfg.servoRefresh = Config::SERVO_REFRESH_RATE;
      cfg.motorRefresh = Config::MOTOR_REFRESH_RATE;

      return cfg;
  }

  //Variables
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

  inline static constexpr ModelConfig m_modelConfig = []() 
  {
    ModelConfig cfg = {};
    for (uint8_t i = 0; i < LedcServo::MAX_LEDC_CHANNELS; i++)
        cfg.outputPins[i] = PIN_UNUSED;
    for (uint8_t i = 0; i < InternalConfig::NUMBER_SERVOS; i++)
        cfg.outputPins[i] = Config::SERVO_PINS[i];
    for (uint8_t i = 0; i < InternalConfig::NUMBER_MOTORS; i++)
        cfg.outputPins[InternalConfig::NUMBER_SERVOS + i] = Config::MOTOR_PINS[i];
    cfg.numberServos = InternalConfig::NUMBER_SERVOS;
    cfg.numberMotors = InternalConfig::NUMBER_MOTORS;
    cfg.servoRefresh = Config::SERVO_REFRESH_RATE;
    cfg.motorRefresh = Config::MOTOR_REFRESH_RATE;
    return cfg;
  }();

protected:
  static constexpr int32_t IDLE_UP = RxBase::MIN_NORMALISED + Config::IDLE_UP_VALUE;
  static constexpr int32_t MIN_THROTTLE = RxBase::MIN_NORMALISED + Config::MIN_THROTTLE_VALUE;

  enum class Actuator : uint8_t
  {
    INDEX_1 = 0U,
    INDEX_2,
    INDEX_3,
    INDEX_4,
    INDEX_5,
    INDEX_6,
    INDEX_7,
    INDEX_8
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

      if constexpr (InternalConfig::DEBUG_OUTPUT)
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
    uint32_t underShoot = 0U;
    uint32_t overShoot  = 0U;

    // Find the SINGLE largest undershoot OR overshoot across ALL motors.
    // We only check overshoot on a motor if it is NOT undershooting (exactly as the original code did).
    for (uint32_t* motor : motors)
    {
        const uint32_t val = *motor;

        if (val < m_rateMinThrottleTicks)
        {
            const uint32_t tempUnder = m_rateMinThrottleTicks - val;
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
  }
};

/**
 * @brief  Plane with wing ailerons/flaperons, rudder and elevator.
 *
 * For this model, Config.hpp must declare:
 *
 *   static constexpr uint8_t SERVO_PINS[] =
 *   {
 *       ESP32S3.OUTPUT_x,   // Servo 1 - left aileron
 *       ESP32S3.OUTPUT_x,   // Servo 2 - right aileron
 *       ESP32S3.OUTPUT_x,   // Servo 3 - elevator
 *       ESP32S3.OUTPUT_x,   // Servo 4 - rudder
 *   };
 *
 *   static constexpr uint8_t MOTOR_PINS[] =
 *   {
 *       ESP32S3.OUTPUT_x,   // Motor 1
 *       ESP32S3.OUTPUT_x,   // Motor 2  (Include always as this plane has optional dual motors)
 *   };
 *
 * 3 position flaps work with this model (Config::USE_FLAPS).
 * Differential thrust works with this model (Config::USE_DIFFERENTIAL_THRUST).
*/
class PlaneFullHouse : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 4U;
  static constexpr uint8_t MOTORS = 2U;

  PlaneFullHouse() : ModelBase(){};
  ~PlaneFullHouse(){};

  static_assert(Config::MODEL_TYPE != ModelType::PLANE_FULL_HOUSE || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: PlaneFullHouse requires 4 servos and 2 motors in Config.hpp.");

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
    writeServos({leftAileronTicks, rightAileronTicks, pitchTicks, yawTicks});
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
* @brief    Plane with wing ailerons/flaperons, with rudder and elevator mixed for V-tail i.e.'Talon' type model. 
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - left aileron
*       ESP32S3.OUTPUT_x,   // Servo 2 - right aileron
*       ESP32S3.OUTPUT_x,   // Servo 3 - elevator
*       ESP32S3.OUTPUT_x,   // Servo 4 - rudder
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2  (Include always as this plane has optional dual motors)
*   };
*
* @note     Channel output map: left aileron, right aileron, Left taileron, right taileron, motor 1, motor 2
* @note     3 position flaps work with this model.
*/
class PlaneFullHouseVTail : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 4U;
  static constexpr uint8_t MOTORS = 2U;

  PlaneFullHouseVTail() : ModelBase(){};
  ~PlaneFullHouseVTail(){};

  static_assert(Config::MODEL_TYPE != ModelType::PLANE_FULL_HOUSE_V_TAIL || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: PlaneFullHouseVTail requires 4 servos and 2 motors in Config.hpp.");

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
    const int32_t yawMinusPitch  = constrain(demands->yaw - demands->pitch,RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const int32_t yawPlusPitch = constrain(demands->yaw + demands->pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED);
    const uint32_t leftAileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollMinusFlap) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightAileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(rollPlusFlap) + (trim->servo2 * getTrimMultiplier()));
    const uint32_t leftTaileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(yawMinusPitch) + (trim->servo3 * getTrimMultiplier()));
    const uint32_t rightTaileronTicks = static_cast<uint32_t>(mapNormalisedServoToTimerTicks(yawPlusPitch) + (trim->servo4 * getTrimMultiplier()));
    writeServos({leftAileronTicks, rightAileronTicks, leftTaileronTicks, rightTaileronTicks});
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
    const uint32_t leftTaileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(yawMinusPitch) + (trim->servo3 * getTrimMultiplier()));
    const uint32_t rightTaileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(yawPlusPitch) + (trim->servo4 * getTrimMultiplier()));
    writeServos({leftAileronTicks, rightAileronTicks, leftTaileronTicks, rightTaileronTicks});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - rudder
*       ESP32S3.OUTPUT_x,   // Servo 2 - elevator
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2  (Include always as this plane has optional dual motors)
*   };
*
* @note     Channel output map: rudder, elevator, motor 1, motor 2
* @note     Both roll and yaw gyro corrections are mixeded to rudder, this makes for better aerobatics but is harder to tune.
*/
class PlaneAdvancedRudderElevator : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 2U;
  static constexpr uint8_t MOTORS = 2U;

  PlaneAdvancedRudderElevator() : ModelBase(){};
  ~PlaneAdvancedRudderElevator(){};

  static_assert(Config::MODEL_TYPE != ModelType::PLANE_ADVANCED_RUDDER_ELEVATOR || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: PlaneAdvancedRudderElevator requires 2 servos and 2 motors in Config.hpp.");

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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - rudder
*       ESP32S3.OUTPUT_x,   // Servo 2 - elevator
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2  (Include always as this plane has optional dual motors)
*   };
*
* @note     Channel output map: rudder, elevator, motor 1, motor 2
*/
class PlaneRudderElevator : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 2U;
  static constexpr uint8_t MOTORS = 2U;

  PlaneRudderElevator() : ModelBase(){};
  ~PlaneRudderElevator(){};

  static_assert(Config::MODEL_TYPE != ModelType::PLANE_RUDDER_ELEVATOR || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: PlaneRudderElevator requires 2 servos and 2 motors in Config.hpp.");

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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - left taileron
*       ESP32S3.OUTPUT_x,   // Servo 2 - right taileron
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2  (Include always as this plane has optional dual motors)
*   };
*
* @note     Channel output map: left taileron, right taileron, motor 1, motor 2
*/
class PlaneVTail : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 2U;
  static constexpr uint8_t MOTORS = 2U;

  PlaneVTail() : ModelBase(){};
  ~PlaneVTail(){};

  static_assert(Config::MODEL_TYPE != ModelType::PLANE_V_TAIL || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: PlaneVTail requires 2 servos and 2 motors in Config.hpp.");

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
    const uint32_t leftTaileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollPlusPitch) + (trim->servo1 * getTrimMultiplier()));
    const uint32_t rightTaileronTicks = static_cast<uint32_t>(mapRateServoToTimerTicks(rollMinusPitch) + (trim->servo2 * getTrimMultiplier()));
    writeServos({leftTaileronTicks, rightTaileronTicks});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - left elevon
*       ESP32S3.OUTPUT_x,   // Servo 2 - right elevon
*       ESP32S3.OUTPUT_x,   // Servo 3 - rudder
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2  (Include always as this plane has optional dual motors)
*   };
*
* @note     Channel output map: left elevon, right elevon, rudder, LEDc unused, motor 1, motor 2
*/
class PlaneFlyingWing : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 3U;
  static constexpr uint8_t MOTORS = 2U;

  PlaneFlyingWing() : ModelBase(){};
  ~PlaneFlyingWing(){};

  static_assert(Config::MODEL_TYPE != ModelType::PLANE_FLYING_WING || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: PlaneFlyingWing requires 3 servos and 2 motors in Config.hpp.");

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
    writeServos({leftElevonTicks, rightElevonTicks, rudderTicks});
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
    writeServos({leftElevonTicks, rightElevonTicks, rudderTicks});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2
*       ESP32S3.OUTPUT_x,   // Motor 3
*       ESP32S3.OUTPUT_x,   // Motor 4
*   };
*
* @note     Channel output map: motor 1, motor 2, motor 3, motor 4
*/
class QuadXCopter : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 0U;
  static constexpr uint8_t MOTORS = 4U;

  QuadXCopter() : ModelBase(){};

  ~QuadXCopter(){};

  static_assert(Config::MODEL_TYPE != ModelType::QUAD_X_COPTER || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: QuadXCopter requires 0 servos and 4 motors in Config.hpp.");

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::INDEX_1), getDefaultMotorTicks(Actuator::INDEX_2), getDefaultMotorTicks(Actuator::INDEX_3), getDefaultMotorTicks(Actuator::INDEX_4)});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2
*       ESP32S3.OUTPUT_x,   // Motor 3
*       ESP32S3.OUTPUT_x,   // Motor 4
*   };
*
* @note     Channel output map: motor 1, motor 2, motor 3, motor 4
*/
class QuadPlusCopter : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 0U;
  static constexpr uint8_t MOTORS = 4U;

  QuadPlusCopter() : ModelBase(){};

  ~QuadPlusCopter(){};

  static_assert(Config::MODEL_TYPE != ModelType::QUAD_P_COPTER || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: QuadPlusCopter requires 0 servos and 4 motors in Config.hpp.");

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::INDEX_1), getDefaultMotorTicks(Actuator::INDEX_2), getDefaultMotorTicks(Actuator::INDEX_3), getDefaultMotorTicks(Actuator::INDEX_4)});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - front servo
*       ESP32S3.OUTPUT_x,   // Servo 2 - back servo
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2
*   };
*
* @note     Channel output map: Front servo, rear servo, motor 1, motor 2
*/
class ChinookCopter : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 2U;
  static constexpr uint8_t MOTORS = 2U;

  ChinookCopter() : ModelBase(){};

  ~ChinookCopter(){};

  static_assert(Config::MODEL_TYPE != ModelType::CHINOOK_COPTER || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: ChinookCopter requires 2 servos and 2 motors in Config.hpp.");

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::INDEX_1), getDefaultMotorTicks(Actuator::INDEX_2)});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - left servo
*       ESP32S3.OUTPUT_x,   // Servo 2 - right servo
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2
*   };
*
* @note     Channel output left servo: right servo, motor 1, motor 2
*/
class BiCopter : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 2U;
  static constexpr uint8_t MOTORS = 2U;

  BiCopter() : ModelBase(){};

  ~BiCopter(){};

  static_assert(Config::MODEL_TYPE != ModelType::BI_COPTER || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: BiCopter requires 2 servos and 2 motors in Config.hpp.");

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::INDEX_1), getDefaultMotorTicks(Actuator::INDEX_2)});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - tail servo
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2
*       ESP32S3.OUTPUT_x,   // Motor 3
*   };
*
* @note     Channel output map: tail servo, motor 1, motor 2, motor 3
*/
class TriCopter : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 1U;
  static constexpr uint8_t MOTORS = 3U;

  TriCopter() : ModelBase(){};

  ~TriCopter(){};

  static_assert(Config::MODEL_TYPE != ModelType::TRI_COPTER || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: TriCopter requires 1 servo and 3 motors in Config.hpp.");

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::INDEX_1), getDefaultMotorTicks(Actuator::INDEX_2), getDefaultMotorTicks(Actuator::INDEX_3)});
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

    multicopterMotorMagic({&motor1, &motor2, &motor3});

    motor1 = constrain(motor1, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor2 = constrain(motor2, getRateMinThrottleTicks(), getMaxMotorTicks());
    motor3 = constrain(motor3, getRateMinThrottleTicks(), getMaxMotorTicks());
    writeMotors({motor1, motor2, motor3});
  };

  virtual void servoMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When disarmed
    const uint32_t servo1 = mapNormalisedServoToTimerTicks(demands->yaw) + (trim->servo1 * getTrimMultiplier());
    writeServos({servo1});
  }

  virtual void servoRateMixer(DemandProcessor::Demands const * const demands, FileSystem::ServoTrims const * const trim) final
  {
    //When armed
    const uint32_t servo1 = mapRateServoToTimerTicks(demands->yaw) + (trim->servo1 * getTrimMultiplier());
    writeServos({servo1});
  }
};



/**
* @brief    Dualcopter class. 
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1 - roll servo
*       ESP32S3.OUTPUT_x,   // Servo 2 - pitch servo
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*       ESP32S3.OUTPUT_x,   // Motor 2
*   };
*
* @note     Channel output map: roll servo, pitch servo, motor 1, motor 2
*/
class DualCopter : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 2U;
  static constexpr uint8_t MOTORS = 2U;

  DualCopter() : ModelBase(){};

  ~DualCopter(){};

  static_assert(Config::MODEL_TYPE != ModelType::DUAL_COPTER || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: DualCopter requires 2 servos and 2 motors in Config.hpp.");

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::INDEX_1), getDefaultMotorTicks(Actuator::INDEX_2)});
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
*
* For this model, Config.hpp must declare:
*
*   static constexpr uint8_t SERVO_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Servo 1
*       ESP32S3.OUTPUT_x,   // Servo 2
*       ESP32S3.OUTPUT_x,   // Servo 3
*       ESP32S3.OUTPUT_x,   // Servo 4
*   };
*
*   static constexpr uint8_t MOTOR_PINS[] =
*   {
*       ESP32S3.OUTPUT_x,   // Motor 1
*   };
*
* @note     Channel output map: servo 1, servo 2, servo 3, servo 4, motor 1
*/
class SingleCopter : public ModelBase
{
public:
  static constexpr uint8_t SERVOS = 4U;
  static constexpr uint8_t MOTORS = 1U;

  SingleCopter() : ModelBase(){};

  ~SingleCopter(){};

  static_assert(Config::MODEL_TYPE != ModelType::SINGLE_COPTER || 
    (InternalConfig::NUMBER_SERVOS == SERVOS && InternalConfig::NUMBER_MOTORS == MOTORS),
    "Configuration Error: SingleCopter requires 4 servos and 1 motor in Config.hpp.");

  virtual void motorMixer(DemandProcessor::Demands const * const demands) final
  {
    writeMotors({getDefaultMotorTicks(Actuator::INDEX_1)});
  }

  virtual void motorRateMixer(DemandProcessor::Demands const * const demands) final
  {
    //Remap throttle to avoid tx stick deadzone between min stick position and and idle up value.
    int32_t throttle = map32(demands->throttle, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, IDLE_UP, RxBase::MAX_NORMALISED);
    //Now remap to work with rate mode
    throttle = map32(throttle, IDLE_UP, RxBase::MAX_NORMALISED, -PIDF::PIDF_MAX_LIMIT, PIDF::PIDF_MAX_LIMIT);

    //Set control mix and convert demands to timer ticks
    uint32_t motor1 = mapRateMotorToTimerTicks(throttle);
    writeMotors({motor1});
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