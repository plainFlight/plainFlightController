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

    /**
    * @brief  Work time of the previous loop as measured by loopRateControl(), before the busy-wait.
    * @note   The loop's spare time is the loop period minus this value.
    * @return Cycle time in microseconds.
    */
    uint32_t getCycleTimeUs() const {return static_cast<uint32_t>(m_cycleTime);};

    /**
    * @brief  Start time of the current loop as recorded by loopRateControl().
    * @return Time since boot in microseconds.
    */
    uint64_t getLoopStartTimeUs() const {return m_loopStartTime;};

    /**
    * @brief  Time between the start of the previous loop and the start of the current loop.
    * @return Loop period in microseconds.
    */
    uint32_t getLoopPeriodUs() const {return static_cast<uint32_t>(m_loopStartTime - m_lastLoopTime);};

  protected:
    #define map32(x, in_min, in_max, out_min, out_max) ((((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min)
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