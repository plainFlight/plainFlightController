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
* @file   Timer.hpp
* @brief  This class contains timer methods to track the passing of a set time period.
*/
#pragma once

#include <inttypes.h>
#include <Arduino.h>


/**
* @class CTimer
*/

class CTimer
{
  public:  
    enum class State : uint8_t
    {
      IDLE = 0U,
      EXPIRED,
      RUNNING,
    };

    CTimer(const uint32_t delayTime);
    void set(const uint32_t delayTime);
    State getState();

  private:
    uint64_t m_expireTime = 0U;  
    State m_state = State::IDLE; 
};
