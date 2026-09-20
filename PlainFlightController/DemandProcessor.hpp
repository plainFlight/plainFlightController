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
* @file   DemandProcessor.hpp
* @brief  This class handles RC commands and makes them into meaningful control demands.
*/
#pragma once

#include <cstdint>
#include <Arduino.h>
#include "Utilities.hpp"
#include "RxBase.hpp"
#include "TelemetryManager.hpp"
#include "ReceiverBearer.hpp"  // instantiates receiver and telemetry bearers
#include "Config.hpp"
#include "Configurator.hpp"
#include "PIDF.hpp"

/**
* @class  StickRespnse
* @note   This class modifies stick response to either make stick inputs feel snappier or duller.
*/
class StickRespnse
{
  public:
    StickRespnse(){};
    ~StickRespnse(){};

    /**
    * @brief    Use a D gain calculation on stick change to increase/decrease stick response. 
    *           This can either make stick inputs feel snappier or duller.
    * @param    A specific stick axis demand (pitch/roll/yaw).
    * @param    Gains for the axis of the stick (pitch/roll/yaw).
    */    
    int32_t stickRespose(const int32_t stickDemand, const PIDF::Gains* const gains)
    {
      //D Term on demand - gives kick for demanded motion inputs
      m_stickResponse = ((static_cast<int64_t>(stickDemand) - static_cast<int64_t>(m_lastStickDemand)) * static_cast<int64_t>(gains->dff));
      m_lastStickDemand = stickDemand;

      //Simple filter to smooth D gain spikes. This will phase shift the signal slightly but values have been optimised.
      m_filteredStickResponse = ((m_filteredStickResponse * WEIGHT_OLD) + m_stickResponse) / FILTER_DIVISOR;

      return m_filteredStickResponse;
    }

  private:
    //Constants
    static constexpr int32_t WEIGHT_OLD = 5;
    static constexpr int32_t FILTER_DIVISOR = WEIGHT_OLD + static_cast<int32_t>(1); 
    //Variables
    int32_t m_lastStickDemand = 0;
    int32_t m_stickResponse = 0;
    int32_t m_filteredStickResponse = 0;
};


/**
* @class  DemandProcessor
* @note   Inherits Utilities class.
*/
class DemandProcessor : public Utilities
{
public:
  enum class FlightState : uint8_t
  {
    //Note: Changing the order of these will have bad consequences
    PASS_THROUGH = 0U,
    RATE,
    SELF_LEVELLED,
    ACRO_TRAINER,
    PROP_HANG,
    WAITING_TO_DISARM,
    AP_WIFI,
    FAULTED,
    CALIBRATE,
    DISARMED,
    FAILSAFE,

  };

  struct Demands
  {
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
    int32_t throttle;
    int32_t flaps;
    bool armed;
    bool headingHold;
    bool propHang;
  };

  //Constants
  static constexpr Demands DEFAULT_DEMANDS = {
      //Used to force outputs to known states
      RxBase::MID_NORMALISED,   //pitch
      RxBase::MID_NORMALISED,   //Roll
      RxBase::MID_NORMALISED,   //Yaw
      RxBase::MIN_NORMALISED,   //Throttle
      RxBase::MIN_NORMALISED,   //Flaps
      false,                    //Armed
      false,                    //Heading Hold
      false,                    //Prop Hang
  };

  DemandProcessor();
  ~DemandProcessor();
  void process(FlightState* const flightState,
                FlightState* const lastFlightState,
                FileSystem::Rates const * const rates,
                FileSystem::MaxAngle const * const maxAngle,
                PIDF::AxisGains const * const gains);
  bool inFailsafeState();
  bool inNeedToDisarmState();
  FlightState getOperatingMode();
  void printData();
  FlightState getDemandedFlightModeFixedWing();
  FlightState getDemandedFlightModeMultiCopter();
  bool isArmed();
  bool throttleIsHigh();
  bool headingHoldActive();
  bool propHangActive();

  /**
  * @brief  Returns the telemetry interface if the active receiver supports it.
  * @return Pointer to ITelemetry implementation, or nullptr.
  */
  ITelemetry* getTelemetry() const { return m_telemetry; };

  Demands const * const getDemands() const {return &m_demand;};
  RxBase::RxPacket const * const getNormalisedRcData() const {return &m_normalisedData;};

private:
  void decodeOperatingMode(FlightState* const flightState, FlightState* const lastFlightState);
  void decodeStickPositions(FlightState const* const flightState, FileSystem::Rates const* const rates, FileSystem::MaxAngle const* const maxAngle, PIDF::AxisGains const* const gains);
  bool wifiApDemanded();

  //Variables
  RxBase::RxPacket m_normalisedData = {0};
  Demands m_demand = DEFAULT_DEMANDS;
  bool m_throttleHigh = false;

  //Objects
  RxBase* radioCtrl = nullptr;
  ITelemetry* m_telemetry = nullptr;
  StickRespnse stickResponsePitch = StickRespnse();
  StickRespnse stickResponseRoll = StickRespnse();
  StickRespnse stickResponseYaw = StickRespnse();
};
