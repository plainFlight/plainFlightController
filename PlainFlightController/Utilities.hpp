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
* @file   Utilities.hpp
* @brief  This class contains methods that all clases may call upon
*/
#pragma once

#include <inttypes.h>
#include <Arduino.h>



/**
 * @class Utilities
 */
 
class Utilities
{
  public:
    Utilities(){};
    ~Utilities(){};

    static constexpr uint64_t LOOP_RATE_US = 1000U;

  protected:
    #define map32(x, in_min, in_max, out_min, out_max) ((((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min)
    float loopRateControl();
    void printLoopRateData();
    float invSqrt(float x);
    float fastAtan2(float y, float x);

  private:
    //LoopRateControl methods data    
    uint64_t m_loopEndTime = 0U;
    uint64_t m_loopStartTime = 0U;
    uint64_t m_lastLoopTime = 0U;
    float m_timeDelta;
    uint64_t m_cycleTime = 0U;
    float m_avCycleTime = 0.0f;
};


class Filter
{
  public:
    float m_alpha = 0.0f;
    float m_average = 0.0f;
    const float m_loopTimeMs;

    Filter(const float loopTime) : m_loopTimeMs(loopTime){};
    ~Filter(){};

    void begin(const float refreshRate, const float responsiveness = 1.0f)
    {
      const float loopTime = static_cast<float>(m_loopTimeMs) / 1000000.0F;
      m_alpha = expf(-loopTime / ((1.0f / refreshRate) * responsiveness));
    }

    void doFilter(const float newValue)
    {
      m_average = (m_alpha * m_average) + ((1.0f - m_alpha) * newValue);
    } 

    float updateFilter(const float newValue)
    {
      m_average = (m_alpha * m_average) + ((1.0f - m_alpha) * newValue);
/*      
      static uint8_t count = 0;

      if (count >= 9)
      {
        Serial.print(m_alpha, 3);
        Serial.print(",");
        Serial.print(newValue,3);
        Serial.print(",");
        Serial.println(m_average,3);
        count=0;
      }
      else
      {
        count++;
      }
*/    
      return m_average; 
    } 

    float getValue(){return m_average;}
    float getAlpha(){return m_alpha;}
};