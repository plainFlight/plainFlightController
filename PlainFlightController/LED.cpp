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
* @file   LED.cpp
* @brief  This class handles LED flash sequences.
*/

#include "LED.hpp"

/**
* @brief    Instantiates an LED output.
*/  
LED::LED(uint32_t ledPin, SinkSource sinkSource)
{
  m_ledPin = ledPin; 
  m_sinkLed = static_cast<bool>(sinkSource); 
}


/**
* @brief    Set the port pin for LED output.
*/  
void 
LED::begin()
{
  pinMode(m_ledPin, OUTPUT);
  sequenceTimer.set(0);
}


/**
* @brief   Plays an LED flash sequence depending upon the flight state passed in. 
*/  
void 
LED::operate(const uint32_t sequence)
{
  if (m_sequenceFinished)
  {
    //Only change sequence once last sequence has finished
    m_sequenceFinished = false;
    //We should never get an error but just good practice to check and handle... 
    m_playSequence = (sequence < m_numSequences) ? sequence: 0U;
  }

  //Play the LED flash sequence
  if (CTimer::State::EXPIRED == sequenceTimer.getState())
  {
    bool state = (m_sinkLed) ? !sequences[m_playSequence].led[m_idx].state : sequences[m_playSequence].led[m_idx].state ;
    digitalWrite(m_ledPin, state);
    sequenceTimer.set(sequences[m_playSequence].led[m_idx].duration);
    m_idx = (++m_idx >= sequences[m_playSequence].size) ? 0U: m_idx;
    m_sequenceFinished = (0U == m_idx) ? true:false;
  }
}