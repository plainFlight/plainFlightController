/* 
* Original File Author: D. Gamble (Github: Cyberslug)
*
* Copyright (c) 2026 P.Cook (alias 'plainFlight')
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
* @file   SerialMonitor.cpp
* @brief  This class implements DEBUG outputs via the serial console port.
*/

#include "SerialMonitor.hpp"
#include "FlightControl.hpp"


/**
* @brief  Called once at the end of FlightControl::begin().
*/
void
SerialMonitor::begin(const FlightControl& /*fc*/)
{
}


/**
* @brief  Called at the end of every loop.
* @param  yaw   demand
*/
void
SerialMonitor::update(const FlightControl& fc)
{
  if constexpr (SOURCE == MonitorSource::LOOP_RATE)
  {
    printLoopRate(fc);
  }
}


/**
* @brief   Rate limits the console output.
* @return  True once every PRINT_PERIOD_MS, and only while the serial port is connected.
*/
bool
SerialMonitor::printDue()
{
  const uint32_t now = millis();

  // Signed difference so the comparison still works when millis() wraps.
  if ((static_cast<int32_t>(now - m_nextPrintMs) >= 0) && Serial)
  {
    m_nextPrintMs = now + PRINT_PERIOD_MS;
    return true;
  }

  return false;
}


/**
* @brief  Loop period, cycle time and average cycle time, in seconds.
* @note   The average is updated every loop; only the printing is rate limited.
*/
void
SerialMonitor::printLoopRate(const FlightControl& fc)
{
  const float cycleTimeUs = static_cast<float>(fc.getCycleTimeUs());
  m_avCycleTimeUs = ((m_avCycleTimeUs * 999.0f) + cycleTimeUs) / 1000.0f;

  if (printDue())
  {
    Serial.print(static_cast<float>(fc.getLoopPeriodUs()) / 1000000.0f, 6);
    Serial.print("\t");
    Serial.print(cycleTimeUs / 1000000.0f, 6);
    Serial.print("\t");
    Serial.println(m_avCycleTimeUs / 1000000.0f, 6);
  }
}