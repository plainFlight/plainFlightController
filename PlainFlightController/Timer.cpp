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
* @file   Timer.cpp
* @brief  This class contains timer methods to track the passing of a set time period.
*/

#include "Timer.hpp"



/**
* @brief    Constructor.
* @param    Initial time delay.
*/ 
CTimer::CTimer(const uint32_t delayTime)
{
  m_expireTime = delayTime;
}


/**
* @brief    Sets the desired time delay.
* @param    Desired time delay to set.
*/ 
void 
CTimer::set(const uint32_t delayTime)
{
  m_expireTime = millis() + delayTime;
  m_state = State::RUNNING;
}


/**
* @brief    Indicates when timer has expired, is running or idle. 
* @return   Timer state when time period has expired.
*/ 
CTimer::State 
CTimer::getState()
{
  if (State::IDLE == m_state)
  {
    return State::IDLE;
  }
  else 
  {
    if (millis() >= m_expireTime) 
    {
      m_state = State::IDLE;
      return State::EXPIRED;    
    }
    else
    {
      return State::RUNNING;
    }
  }
}