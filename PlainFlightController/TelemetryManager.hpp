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
* @file   TelemetryManager.hpp
* @brief  Rate-gating orchestrator for outgoing telemetry transmissions.
*
* Role in the telemetry path
* --------------------------
* The flight controller holds a TelemetryManager instance and calls update()
* on each iteration of operate().  The manager decides whether each telemetry
* type is due for transmission based on per-type timers, then delegates the
* actual serialisation and UART write to the ITelemetry implementation it was
* given at construction time.
*
* The manager has no knowledge of the underlying protocol.  It knows only the
* ITelemetry interface and the raw values passed to update().  Adding a new
* telemetry type requires adding a timer here and a corresponding send call in
* update(); no other structural change is needed.
*
* Null-pointer capability guard
* ------------------------------
* When constructed with a nullptr telemetry pointer (the case for a receiver
* with no downlink path, e.g. SBus), update() returns immediately after a
* single null check.  No virtual dispatch, no timer evaluation, no work.
*
* Timer initialisation
* --------------------
* The battery timer is initialised with period 0, which causes CTimer to
* report EXPIRED immediately.  The first call to update() after power-on
* therefore transmits without waiting a full period, getting data to the
* transmitter as early as possible.
*/

#pragma once

#include <inttypes.h>
#include "Timer.hpp"
#include "ITelemetry.hpp"
#include "GnssDriver.hpp"


/**
* @class  TelemetryManager
* @brief  Orchestrates rate-limited transmission of telemetry data.
*
* Constructed once at flight-controller start-up.  Thereafter update() is
* called every main-loop iteration when telemetry is enabled.
*/
class TelemetryManager
{
  public:

        /**
    * @brief   Default constructor.
    * @details Required for instantiation as a member of FlightControl.
    *          Initialises pointers to nullptr and periods to 0.
    */
    TelemetryManager();

    /**
    * @brief   Initialise the manager with a telemetry implementation.
    * @param   telemetry        Pointer to the telemetry implementation (e.g. Crsf).
    * @param   batteryPeriodMs  Minimum interval between battery telemetry frames, ms.
    * @param   gnssPeriodMs     Minimum interval between gnss telemetry frames, ms.
    */
    void begin(ITelemetry* telemetry, uint32_t batteryPeriodMs, uint32_t gnssPeriodMs);

    /**
    * @brief   Push the current battery voltage and transmit if the timer has elapsed.
    * @details Returns immediately if the telemetry pointer is nullptr.  Otherwise
    *          checks whether the battery timer has expired; if so, calls
    *          sendBatteryTelemetry() and restarts the timer.  No serialisation
    *          work occurs on iterations where the timer has not expired.
    * @param   voltageVolts  Current battery pack voltage in volts.
    */
    void updateBattery(const float voltageVolts);

    /**
    * @brief   Push the current gnss data and transmit if the timer has elapsed.
    * @details Returns immediately if the telemetry pointer is nullptr.  Otherwise
    *          checks whether the gnss timer has expired; if so, calls
    *          sendGnssTelemetry() and restarts the timer.  No serialisation
    *          work occurs on iterations where the timer has not expired.
    * @param   data  Current GnssData packet.
    */
    void updateGnss(const GnssData& data);

  private:

    /** Telemetry implementation. nullptr → receiver has no downlink; update() is a no-op. */
    ITelemetry* m_telemetry      = nullptr;

    /** Minimum transmit interval for battery frames, in milliseconds. */
    uint32_t    m_batteryPeriodMs = 0U;

    /** Timer that gates battery telemetry transmissions. */
    CTimer      m_batteryTimer    = CTimer(0U);

    /** Minimum transmit interval for gnss frames, in milliseconds. */
    uint32_t    m_gnssPeriodMs     = 0U;

    /** Timer that gates gnss telemetry transmissions. */
    CTimer      m_gnssTimer        = CTimer(0U);
};