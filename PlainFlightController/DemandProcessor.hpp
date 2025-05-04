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
#include "SBus.hpp"
#include "Config.hpp"
#include "Configurator.hpp"


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

    enum class SwitchPosition : uint32_t
    {
      IS_HIGH = 0U,
      IS_MIDDLE,
      IS_LOW,
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
          SBus::MID_NORMALISED_US,  //pitch
          SBus::MID_NORMALISED_US,  //Roll
          SBus::MID_NORMALISED_US,  //Yaw
          SBus::MIN_NORMALISED_US,  //Throttle
          SBus::MIN_NORMALISED_US,  //Flaps
          false,                    //Armed
          false,                    //Heading Hold
          false,                    //Prop Hang
          };

    DemandProcessor();
    void process(FlightState* const flightState, 
                  FlightState* const lastFlightState, 
                  FileSystem::Rates const * const rates, 
                  FileSystem::MaxAngle const * const maxAngle);
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
    Demands const * const getDemands() const {return &m_demand;};

  private:
    void decodeOperatingMode(FlightState* const flightState, FlightState* const lastFlightState);
    SwitchPosition get2PosSwitch(uint32_t aChannel);
    SwitchPosition get3PosSwitch(uint32_t aChannel);
    void docodeStickPositions(FlightState const * const flightState, FileSystem::Rates const * const rates, FileSystem::MaxAngle const * const maxAngle);   
    bool wifiApDemanded();    

    //Variables
    SBus::SbusPacket m_sBus = {0};
    Demands m_demand = DEFAULT_DEMANDS;
    bool m_throttleHigh = false;

    //Objects
    SBus radioCtrl = SBus(Config::SBUS_UART, Config::SBUS_RX, Config::SBUS_TX);
};
