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
* @file   BearerFactory.hpp
* @brief  This file contains enums and structures that are used to select and 
*         instantiate the Receiver and Telemetry bearers 
*/

#pragma once

#include "RxBase.hpp"
#include "ITelemetry.hpp"
#include "Config.hpp"
#include "InternalConfig.hpp"
#include "Crsf.hpp"
#include "SBus.hpp"
// #include "EspNow.hpp"   // add once finalised; ESPNOW branches below are stubs

/**
* @class  BearerFactory
* @brief  Selects and instantiates the receiver and telemetry bearers based
*         on Config::RECEIVER_TYPE / Config::TELEMETRY_TYPE.
* @note   Adding a new bearer requires a change here only - no other file
*         needs to know which receiver types exist.
*/
class BearerFactory
{
  public:
    // Adding a new receiver bearer requires: adding it to
    // ReceiverType and/or TelemetryType in CommonTypes.hpp, then a branch here.
    static void create(RxBase** outReceiver, ITelemetry** outTelemetry)
    {
      if constexpr (Config::RECEIVER_TYPE == ReceiverType::CRSF)
      {
        Crsf* crsf = new Crsf();
        *outReceiver = crsf;
        *outTelemetry = (Config::TELEMETRY_TYPE == TelemetryType::NONE) ? nullptr : crsf;
      }
      else if constexpr (Config::RECEIVER_TYPE == ReceiverType::SBUS)
      {
        *outReceiver = new SBus();
        *outTelemetry = nullptr;   // SBus has no telemetry path; independent
                                    // bearer support (e.g. ESPNOW) pending
      }
    }
};