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
* @file   ITelemetry.hpp
* @brief  Pure abstract interface for telemetry transmission to an RC transmitter.
*
* Adding a new telemetry type
* ---------------------------
* Add a virtual method here, implement it in Crsf (and any other implementor),
* add the corresponding codec frame builder, and add a timer in TelemetryManager.
* No other files require changes.
*/

#pragma once

#include "GnssTypes.hpp"
/**
* @class  ITelemetry
* @brief  Abstract interface for sending telemetry data to an RC transmitter.
*
* Implementing classes (e.g. Crsf) inherit from both RxBase and ITelemetry.
* The flight controller calls these methods without knowing which protocol or
* transport is in use.
*/
class ITelemetry
{
  public:

    /**
    * @brief  Virtual destructor.
    *         Ensures correct cleanup when destroying through a base-class pointer,
    *         even though ITelemetry holds no data members of its own.
    */
    virtual ~ITelemetry() = default;

    /**
    * @brief   Send battery voltage telemetry to the RC transmitter.
    * @details The implementing class converts the supplied voltage into its own
    *          wire format and transmits it.  All scaling happens inside the 
    *          implementation; the caller passes only the raw float it already has.
    * @param   voltageVolts  Battery pack voltage in volts (e.g. 12.6f).
    */
    virtual void sendBatteryTelemetry(const float& voltageVolts) = 0;

    /**
    * @brief   Send gnss packet telemetry to the RC transmitter.
    * @details The implementing class converts the supplied data into its own
    *          wire format and transmits it.  All scaling happens inside the 
    *          implementation; the caller passes only the raw data it already has.
    * @param   data  A GnssData structure.
    */
    virtual void sendGnssTelemetry(const GnssData& data) = 0;


  protected:

    /**
    * @brief  Protected default constructor.
    *         Prevents direct instantiation; only derived classes may be constructed.
    */
    ITelemetry() = default;
};