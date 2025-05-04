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
* @brief  This class controls one servo.
*/
#pragma once

#include <Arduino.h>
#include <cstdint>
#include "Utilities.hpp"



/**
* @class LedcServo
*/

class LedcServo : public Utilities
{
  public:
    enum class RefreshRate : uint32_t
    {
      IS_50Hz = 50U,
      IS_100Hz = 100U,
      IS_150Hz = 150U,
      IS_200Hz = 200U,
      IS_250Hz = 250U,
      IS_300Hz = 300U,
      IS_350Hz = 350U,
      IS_ONESHOT125 = 2000U, 
    };

    static constexpr uint32_t MAX_MICRO_SECONDS = 2000U;
    static constexpr uint32_t MID_MICRO_SECONDS = 1500U;
    static constexpr uint32_t MIN_MICRO_SECONDS = 1000U;
    static constexpr uint32_t CALIBRATE_ESC_DELAY = 5000;

    LedcServo(const uint8_t pwmPin, const RefreshRate refreshRate, const uint32_t initialMicroSeconds);
    bool begin();
    void setTimerTicks(const uint32_t requiredTicks);
    void debug() const;
    bool hasFaulted() const;
    //uint32_t getDefaultMicroSeconds() const;
    uint32_t getCurrentTimerTicks() const;
    uint32_t getMinTimerTicks() const;
    uint32_t getMaxTimerTicks() const;
    uint32_t getDefaultTimerTicks() const {return m_defaultTimerTicks;}
    int32_t getTrimMultiplier() const {return m_trimMultiplier;}    

  private:
    static constexpr int32_t LEDC_BIT_RESOLUTION  = 14;
    static constexpr int32_t PWM_RESOLUTION       = 16384;
    //Hard coding OneShot125 as fine adjsutments may be required to end points
    static constexpr int32_t ONESHOT125_MIN_TICKS = 4090;
    static constexpr int32_t ONESHOT125_MAX_TICKS = 8192;

    //Variables
    uint8_t m_pwmPin;
    uint32_t m_refreshRate;
    uint32_t m_timerTicks;
    uint32_t m_defaultTimerTicks;
    uint32_t m_minTicks;
    uint32_t m_maxTicks;
    int32_t m_trimMultiplier;    
};