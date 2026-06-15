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
* @brief   Construct a TelemetryManager.
* @details The battery timer is initialised via the member default
*          (CTimer(0U)) so it reports EXPIRED on the first update() call,
*          transmitting immediately rather than waiting a full period.
*          This matches the pattern used by the loss-of-comms timers in
*          Crsf and SBus.
* @param   telemetry        Telemetry implementation pointer (may be nullptr).
* @param   batteryPeriodMs  Battery transmit period in milliseconds.
*/
TelemetryManager::TelemetryManager(ITelemetry* const telemetry,
                                   const uint32_t    batteryPeriodMs)
  : m_telemetry(telemetry),
    m_batteryPeriodMs(batteryPeriodMs)
{
}

/**
* @brief   Default constructor.
*/
TelemetryManager::TelemetryManager()
  : m_telemetry(nullptr),
    m_batteryPeriodMs(0U)
{
}

/**
* @brief   Initialisation.
*/
void
TelemetryManager::begin(ITelemetry* telemetry, uint32_t batteryPeriodMs)
{
  m_telemetry = telemetry;
  m_batteryPeriodMs = batteryPeriodMs;
  m_batteryTimer.set(0U); // Force immediate expire on first update() call.
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
TelemetryManager::update(const float voltageVolts)
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