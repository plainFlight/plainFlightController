/* 
* Copyright (c) 2023,2025 P.Cook (alias 'plainFlight')
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

/*
* PIDF class.
*/

#include "PIDF.hpp"
#include <Arduino.h>



/**
* @brief    Calculates the PIDF.
* @param    setPoint  The demanded position.
* @param    actualPoint   The actual position.
* @param    gains   Pointer to structure of gains for this PIDF calculation.
*/
int32_t 
PIDF::pidfController(const int32_t setPoint, const int32_t actualPoint, const Gains* const gains)
{
  m_error = setPoint - actualPoint;

  //P erm calculations
  m_pTerm = static_cast<int64_t>(m_error) * static_cast<int64_t>(gains->p);
  m_iTerm += (static_cast<int64_t>(m_error) * static_cast<int64_t>(gains->i)) / I_TERM_DENOMINATOR;

  if (abs(m_iTerm) > I_WIND_UP_LIMIT)
  {
    //Limit iTerm to what we need
    if(0 > m_iTerm)
    {
      //iTerm gone negative
      m_iTerm = 0 - I_WIND_UP_LIMIT;
    }
    else
    {
      m_iTerm = I_WIND_UP_LIMIT;
    }
  }

  //D term calculations
  m_dTerm = (static_cast<int64_t>(m_error) - static_cast<int64_t>(m_dLastError)) * static_cast<int64_t>(gains->d);

  if (abs(m_dTerm) > D_WIND_UP_LIMIT)
  {
    if (0 > m_dTerm)
    {
      //dTerm gone negative
      m_dTerm = 0 - D_WIND_UP_LIMIT;
    }
    else
    {
      m_dTerm = D_WIND_UP_LIMIT;
    }
  }

  m_dLastError = static_cast<int64_t>(m_error);   

  //D feed forward (DFF) term calculations
  m_dFFTerm = (setPoint - m_lastSetPoint) * static_cast<int64_t>(gains->dff);
  m_lastSetPoint = static_cast<int64_t>(setPoint);

  if (abs(m_dFFTerm) > DFF_WIND_UP_LIMIT)
  {
    if (0 > m_dFFTerm)
    {
      //dTerm gone negative
      m_dFFTerm = 0 - DFF_WIND_UP_LIMIT;
    }
    else
    {
      m_dFFTerm = DFF_WIND_UP_LIMIT;
    }
  }

  m_dFFTermFiltered = ((m_dFFTermFiltered * WEIGHTED_OLD) + m_dFFTerm) / DFF_FILTER_DENOMINATOR;  


  //FF term calculation
  m_fFTerm = static_cast<int64_t>(setPoint * gains->ff);

  //Sum PID terms
  m_pidTerm =  (m_pTerm + m_iTerm + m_dTerm + m_fFTerm + m_dFFTermFiltered) / PIDF_TERM_DENOMINATOR; 
  const uint32_t pidTerm = static_cast<int32_t>(constrain(m_pidTerm, -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT));
      
  return pidTerm;
}


/**
* @brief    Used to zero out igain.
*/
void 
PIDF::iTermReset()
{
 m_iTerm = 0;
}
