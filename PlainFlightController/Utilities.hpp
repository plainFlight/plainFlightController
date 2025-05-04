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

  protected:
    int32_t map32(const int32_t x, const int32_t in_min, const int32_t in_max, const int32_t out_min, const int32_t out_max);
    float loopRateControl();
    void printLoopRateData();
    float invSqrt(float x);
    float fastAtan2(float y, float x);

  private:
    //LoopRateControl methods data
    static constexpr uint64_t LOOP_RATE_US = 1000U;
    uint64_t m_loopEndTime = 0U;
    uint64_t m_loopStartTime = 0U;
    uint64_t m_lastLoopTime = 0U;
    float m_timeDelta;
    uint64_t m_cycleTime = 0U;
    float m_avCycleTime = 0.0f;
};