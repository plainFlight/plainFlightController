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
* @file   TelemetryManager.cpp
* @brief  Rate-gating orchestrator for outgoing telemetry transmissions.
*/

#include "TelemetryManager.hpp"


/**
* @brief   Default constructor.
*/
TelemetryManager::TelemetryManager()
  : m_telemetry(nullptr),
    m_batteryPeriodMs(0U),
    m_gnssPeriodMs(0U)
{
}

/**
* @brief   Initialisation.
*/
void
TelemetryManager::begin(ITelemetry* telemetry, uint32_t batteryPeriodMs, uint32_t gnssPeriodMs)
{
  m_telemetry = telemetry;
  m_batteryPeriodMs = batteryPeriodMs;
  m_batteryTimer.set(0U); // Force immediate expire on first update() call.
  m_gnssPeriodMs = gnssPeriodMs;
  m_gnssTimer.set(0U); // Force immediate expire on first update() call.
}


/**
* @brief   Check the battery timer and transmit if it has elapsed.
* @details The null-pointer guard is the sole mechanism for handling receivers
*          with no telemetry capability.  When telemetryCtrl is nullptr the
*          function returns without touching the timer or calling any virtual
*          method.
*
*          When telemetryCtrl is valid: if the battery timer has expired, the
*          voltage value is forwarded to sendBatteryTelemetry() and the timer
*          is restarted.  The called method does the serialisation and UART
*          write; this method owns only the timing decision.
*
* @param   voltageVolts  Current battery pack voltage in volts.
*/
void
TelemetryManager::updateBattery(const float voltageVolts)
{
  // Early exit: no telemetry path available (e.g. SBus receiver).
  if (nullptr == m_telemetry)
  {
    return;
  }

  if (CTimer::State::EXPIRED == m_batteryTimer.getState())
  {
    m_telemetry->sendBatteryTelemetry(voltageVolts);
    m_batteryTimer.set(static_cast<uint64_t>(m_batteryPeriodMs));
  }
}

/**
* @brief   Push the current gnss data and transmit if the timer has elapsed.
* @details Returns immediately if the telemetry pointer is nullptr.  Otherwise
*          checks whether the gnss timer has expired; if so, calls
*          sendGnssTelemetry() and restarts the timer.  No serialisation
*          work occurs on iterations where the timer has not expired.
* @param   data  Current GnssData packet.
*/
void
TelemetryManager::updateGnss(const GnssData& data)
{
  // Early exit: no telemetry path available (e.g. SBus receiver).
  if (nullptr == m_telemetry)
  {
    return;
  }

  if (CTimer::State::EXPIRED == m_gnssTimer.getState())
  {
    m_telemetry->sendGnssTelemetry(data);
    m_gnssTimer.set(static_cast<uint64_t>(m_gnssPeriodMs));
  }
}
