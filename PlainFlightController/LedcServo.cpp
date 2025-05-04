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
* @file   LedcServo.hpp
* @brief  This file controls one servo.
*/

#include "LedcServo.hpp"


/**
* @brief  Constructor for LedcServo class. 
* @param  pin to output servo PWM/pulse on.
* @param  Servo refresh rate Hz. 
* @param  Inital value and failsafe position of servo. 1500ms is servo centered.
* @note   Recommend setting throttle channel initial/failsafe value to 1000ms.
* @note   Set analogue servos to 50Hz only! Digital servos typically 150-250Hz, check servo datasheet before setting!
* @note   Too high a refresh rate may permanently damage servo(s)!
*/  
LedcServo::LedcServo(const uint8_t pwmPin, RefreshRate refreshRate, const uint32_t initialMicroSeconds)
{
  //Initialise member variables.
  m_pwmPin = pwmPin;
  m_refreshRate = static_cast<uint32_t>(refreshRate); 

  if (RefreshRate::IS_ONESHOT125 == refreshRate)
  {
    //Hard code OneShot125 as endpoints may need fine adjustment
    m_minTicks = ONESHOT125_MIN_TICKS;
    m_maxTicks = ONESHOT125_MAX_TICKS;
    m_defaultTimerTicks = ONESHOT125_MIN_TICKS;
  }
  else
  {
    const int32_t microSeconds = constrain(initialMicroSeconds, MIN_MICRO_SECONDS, MAX_MICRO_SECONDS); 
    //Below calculation assumes refresh rate is in multiples of 50
    m_trimMultiplier = static_cast<int32_t>(refreshRate) / static_cast<int32_t>(RefreshRate::IS_50Hz);
    //Calculate how many timer ticks we need to make 1ms & 2ms at chosen refresh rate...
    const float period = static_cast<float>(MIN_MICRO_SECONDS) / static_cast<float>(m_refreshRate);
    const uint32_t timerDivisor = static_cast<uint32_t>(((static_cast<float>(PWM_RESOLUTION) / period) + 0.5f));
    m_minTicks = timerDivisor;
    m_maxTicks = timerDivisor * 2U;
    //Convert micro seconds to timer ticks
    m_defaultTimerTicks = map32(microSeconds, MIN_MICRO_SECONDS, MAX_MICRO_SECONDS, m_minTicks, m_maxTicks);
  }
}


/**
* @brief  Constructor for LedcServo class. 
* @return True upon initialisation success.
*/
bool 
LedcServo::begin()
{
  bool success = ledcAttach(m_pwmPin, m_refreshRate, LEDC_BIT_RESOLUTION);

  if (!success)
  {
    Serial.print("LEDc attach failure, pin: ");
    Serial.println(m_pwmPin);
    return false;
  }
  else
  {
    setTimerTicks(m_defaultTimerTicks);
    return true;
  }  
}


/**
* @brief  Sets the required position of the servo.
* @param  The value in LEDc timer ticks seconds of required servo position.  
*/
void 
LedcServo::setTimerTicks(const uint32_t requiredTicks)
{
  m_timerTicks = constrain(requiredTicks, m_minTicks, m_maxTicks);
  ledcWrite(m_pwmPin, m_timerTicks);
}


/**
* @brief  
* @return 
*/
uint32_t 
LedcServo::getCurrentTimerTicks() const
{
  return   m_timerTicks;
}


/**
* @brief  
* @return 
*/
uint32_t 
LedcServo::getMinTimerTicks() const
{
  return m_minTicks;
}


/**
* @brief  
* @return 
*/
uint32_t 
LedcServo::getMaxTimerTicks() const
{
  return m_maxTicks;
}


/**
* @brief  Outputs data console.
*/
void 
LedcServo::debug() const
{
  Serial.print(m_refreshRate);
  Serial.print(", ");
  Serial.print(m_minTicks);
  Serial.print(", ");
  Serial.print(m_maxTicks);  
  Serial.print(", ");
  Serial.print(m_defaultTimerTicks);  
  Serial.print(", ");
  Serial.println(m_timerTicks);  
}