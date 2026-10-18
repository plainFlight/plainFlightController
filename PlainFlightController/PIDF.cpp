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
PIDF::pidfController(const int32_t setPoint, const int32_t actualPoint)
{
  m_error = setPoint - actualPoint;

  //P erm calculations
  m_pTerm = static_cast<int64_t>(m_error) * static_cast<int64_t>(m_gains->p);

  if (m_iTermIsEnabled)
  {
    m_iTerm += (static_cast<int64_t>(m_error) * static_cast<int64_t>(m_gains->i)) / I_TERM_DENOMINATOR;

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
  }
  else
  {
    m_iTerm = 0;
  }

  //D term calculations
  if (0 < m_gains->d)
  {
    //Positive D gain gives 'D On Error' calculation - Damps gyro maesurement but has side effect of affecting stick acceleration.
    m_dTerm = (static_cast<int64_t>(m_error) - static_cast<int64_t>(m_dLastError)) * static_cast<int64_t>(m_gains->d);
    m_dLastError = static_cast<int64_t>(m_error);
  }
  else
  {
    //Negative D gain gives 'D On Measurement' calculation - Damps gyro measurement only, can tune stick acceleration via DFF independently.
    m_dTerm = (static_cast<int64_t>(actualPoint) - static_cast<int64_t>(m_dLastError)) * static_cast<int64_t>(m_gains->d);
    m_dLastError = static_cast<int64_t>(actualPoint); 
  }

  calcStickAcceleration(setPoint);

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

  //FF term calculation
  m_fFTerm = static_cast<int64_t>(setPoint * m_gains->ff);

  //Sum PID terms
  m_pidTerm =  (m_pTerm + m_iTerm + m_dTerm + m_fFTerm + m_dFFTermFiltered) / PIDF_TERM_DENOMINATOR; 
  const int32_t pidTerm = static_cast<int32_t>(constrain(m_pidTerm, -PIDF_MAX_LIMIT, PIDF_MAX_LIMIT));

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


/**
* @brief    Calculates a D term based on stick accelerations. Used to give snappier responses to stick inputs.
* @param    stickDemand The Tx stick demand
*/
void
inline PIDF::calcStickAcceleration(const int32_t stickDemand)
{
  //D feed forward (DFF) term calculations
  m_stickFF = (stickDemand - m_lastStickDemand) * m_gains->dff;
  m_lastStickDemand = stickDemand;
  m_dFFTermFiltered = ((m_dFFTermFiltered * WEIGHTED_OLD) + m_stickFF) / DFF_FILTER_DENOMINATOR;
}
