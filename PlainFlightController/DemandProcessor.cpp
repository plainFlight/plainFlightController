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
* @file   DemandProcessor.cpp
* @brief  This class handles RC commands and makes them into meaningful control demands.
*/

#include "DemandProcessor.hpp"


/**
* @brief    DemandProcessor constructor   
*/
DemandProcessor::DemandProcessor()
{
  // Add instantiation code here for new receiver protocols
  if constexpr (Config::RECEIVER_TYPE == RxBase::ReceiverType::SBUS)
  {
    radioCtrl = new SBus(Config::RECEIVER_UART, Config::RECEIVER_RX, Config::RECEIVER_TX);
  } else if constexpr (Config::RECEIVER_TYPE == RxBase::ReceiverType::CRSF)
  {
    radioCtrl = new Crsf(Config::RECEIVER_UART, Config::RECEIVER_RX, Config::RECEIVER_TX);
  }


  m_normalisedData = radioCtrl->getData();  //Copy across initial Sbus data state i.e. failsafe flag state
}


/**
* @brief    DemandProcessor destructor   
*/
DemandProcessor::~DemandProcessor()
{
  delete radioCtrl;
  radioCtrl = nullptr;
}

/**
* @brief    Processes RC data to determine operating mode and any drive demands.
* @param    flightState   Pointer to the current flight state.
* @param    lastFlightState   Pointer to the previous flight state.
* @param    modelCategory Used as multicopters must not have pass through state
*/
void
DemandProcessor::process(FlightState* const flightState,
                         FlightState* const lastFlightState,
                         FileSystem::Rates const * const rates,
                         FileSystem::MaxAngle const * const maxAngle)
{
  if (radioCtrl->getDemands())  //Rx sbus processing
  {
    //New sbus packet received so process it
    m_normalisedData = radioCtrl->getData();
    decodeOperatingMode(flightState, lastFlightState);
    decodeStickPositions(flightState, rates, maxAngle);
  }
  else
  {
    decodeOperatingMode(flightState, lastFlightState);
  }
}


/**
* @brief    Decode transmitter stick positions into meaningful demands.
* @param    flightState   Pointer to the current flight state.
*/
void
DemandProcessor::decodeStickPositions(FlightState const* const flightState, FileSystem::Rates const* const rates, FileSystem::MaxAngle const* const maxAngle)
{
  //Normalise stick commands to signed values that we can work with
  m_demand.pitch = radioCtrl->getChannel(m_normalisedData, RxBase::ChannelName::PITCH);

  if (Config::TX_DEADBAND_NORM > abs(m_demand.pitch))
  {
    m_demand.pitch = 0;
  }

  m_demand.roll = radioCtrl->getChannel(m_normalisedData, RxBase::ChannelName::ROLL);

  if (Config::TX_DEADBAND_NORM > abs(m_demand.roll))
  {
    m_demand.roll = 0;
  }

  m_demand.yaw = radioCtrl->getChannel(m_normalisedData, RxBase::ChannelName::YAW);

  if (Config::TX_DEADBAND_NORM > abs(m_demand.yaw))
  {
    m_demand.yaw = 0;
  }

  m_demand.throttle = radioCtrl->getChannel(m_normalisedData, RxBase::ChannelName::THROTTLE);
  m_demand.flaps = radioCtrl->getChannel(m_normalisedData, RxBase::ChannelName::AUX1);

  switch (*flightState)
  {
  case FlightState::ACRO_TRAINER:
    //Acro trainer uses degrees per sec, when levelling the angle demands are zero.
    //Intentional fall through
  case FlightState::RATE:
    //Map control inputs to degrees per second.
    m_demand.pitch = map32(m_demand.pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -rates->pitch, rates->pitch);
    m_demand.roll = map32(m_demand.roll, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -rates->roll, rates->roll);
    m_demand.yaw = map32(m_demand.yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -rates->yaw, rates->yaw);
    break;

  case FlightState::SELF_LEVELLED:
    //Map control inputs to degrees.
    m_demand.pitch = map32(m_demand.pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -maxAngle->pitch, maxAngle->pitch);
    m_demand.roll = map32(m_demand.roll, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -maxAngle->roll, maxAngle->roll);
    //Yaw still works in degrees per second.
    m_demand.yaw = map32(m_demand.yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -rates->yaw, rates->yaw);
    break;

  case FlightState::PROP_HANG:
    //Map control inputs to degrees.
    m_demand.pitch = map32(m_demand.pitch, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -maxAngle->pitch, maxAngle->pitch);
    if constexpr (Config::PROP_HANG_TAIL_SITTER_MODE)
    {
      //Roll works in rate mode
      m_demand.roll = map32(m_demand.roll, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -maxAngle->roll, maxAngle->roll);
      //Yaw set to max roll angle
      m_demand.yaw = map32(m_demand.yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -rates->roll, rates->roll);
    }
    else
    {
      //Roll works in rate mode
      m_demand.roll = map32(m_demand.roll, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -rates->roll, rates->roll);
      //Yaw set to max roll angle
      m_demand.yaw = map32(m_demand.yaw, RxBase::MIN_NORMALISED, RxBase::MAX_NORMALISED, -maxAngle->roll, maxAngle->roll);
    }
    break;

  case FlightState::FAILSAFE:
    m_demand = DEFAULT_DEMANDS;
    break;

  default:
    //Assume pass through for all other states.
    break;
  }
}


/**
* @brief  Determines what state the radio control system is in  
* @param    flightState   Pointer to the current flight state.
* @param    lastFlightState   Pointer to the previous flight state.
*/
void
DemandProcessor::decodeOperatingMode(FlightState* const flightState, FlightState* const lastFlightState)
{
  FlightState demandedFlightState = *flightState;
  m_demand.armed = RxBase::isSwitchHigh(m_normalisedData.ch[static_cast<uint32_t>(RxBase::ChannelName::ARM)]);
  m_throttleHigh = (RxBase::LOW_THROTTLE_NORM < m_normalisedData.ch[static_cast<uint32_t>(RxBase::ChannelName::THROTTLE)]);

  if (FlightState::CALIBRATE == demandedFlightState)
  {
    ; //Let calibration complete
  }
  else if (radioCtrl->hasLostCommunications() || m_normalisedData.failsafe)
  {
    demandedFlightState = FlightState::FAILSAFE;
  }
  else if (((FlightState::FAILSAFE == *lastFlightState)
          || (FlightState::CALIBRATE == *lastFlightState) 
          || (FlightState::WAITING_TO_DISARM == *flightState))
           && m_demand.armed)
  {
    //Failsafe set, or no longer set but Tx armed when exiting failsafe
    demandedFlightState = FlightState::WAITING_TO_DISARM;
  }
  else if (m_demand.armed)
  {
    DemandProcessor::FlightState demandedFlightMode;

    if constexpr(Config::MODEL_IS_FIXED_WING)
    {
      demandedFlightMode = getDemandedFlightModeFixedWing();
    }
    else
    {
      demandedFlightMode = getDemandedFlightModeMultiCopter();
    }

    //If throttle is high and last state not a normal flight mode then wait to disarm, otherwise allow demanded flight state
    demandedFlightState = (m_throttleHigh && (*lastFlightState > FlightState::PROP_HANG)) ? FlightState::WAITING_TO_DISARM : demandedFlightMode;
  }
  else if (wifiApDemanded())
  {
    demandedFlightState = FlightState::AP_WIFI;
  }
  else if (FlightState::AP_WIFI == demandedFlightState)
  {
    //Wifi no longer demanded so reset processor to force clean start and file read.
    ESP.restart();
  }
  else
  {
    if (FlightState::DISARMED != *flightState)
    {
      demandedFlightState = FlightState::DISARMED;
    }
  }

  if (*flightState != demandedFlightState)
  {
    *lastFlightState = *flightState;
    *flightState = demandedFlightState;
  }
}


/**
* @brief  Determines if the throttle is not low.  
* @return True when throttle is high.
*/
bool
DemandProcessor::throttleIsHigh()
{
  return m_throttleHigh;
}


/**
* @brief  Determines if wifi configurator mode is being demanded 
* @return True when disamred and throttle is high.
*/
bool
DemandProcessor::wifiApDemanded()
{
  return ((!m_demand.armed) && RxBase::isSwitchHigh(m_normalisedData.ch[static_cast<uint32_t>(RxBase::ChannelName::THROTTLE)]));
}


/**
* @brief    Indicates the RC demanded state of the arming switch.
* @return   True when Tx is armed.
*/
bool
DemandProcessor::isArmed()
{
  return m_demand.armed;
}


/**
* @brief    Determines the RC demanded flight mode for fixed wing aircraft.  
* @return   FlightState   The demanded flight state.
*/
DemandProcessor::FlightState
DemandProcessor::getDemandedFlightModeFixedWing()
{
  if constexpr(Config::USE_PROP_HANG_MODE)
  {
    m_demand.propHang = RxBase::isSwitchHigh(m_normalisedData.ch[static_cast<uint32_t>(RxBase::ChannelName::AUX3)]);

    if (m_demand.propHang)
    {
      return FlightState::PROP_HANG;
    }
  }

  const RxBase::SwitchPosition modeSwitchPosition = RxBase::getSwitch3Position(m_normalisedData.ch[static_cast<uint32_t>(RxBase::ChannelName::MODE)]);

  switch (modeSwitchPosition)
  {
  case RxBase::SwitchPosition::SW_HIGH:
  {
    if constexpr (Config::USE_ACRO_TRAINER)
    {
      return FlightState::ACRO_TRAINER;
    }
    else
    {
      return FlightState::SELF_LEVELLED;
    }
  }
  break;

  case RxBase::SwitchPosition::SW_MID:
    return FlightState::RATE;
    break;

  default:  //Intentional fall through
  case RxBase::SwitchPosition::SW_LOW:
    return FlightState::PASS_THROUGH;
    break;
  }
}


/**
* @brief    Determines the RC demanded flight mode for multicopter aircraft.  
* @return   FlightState   The demanded flight state.
*/
DemandProcessor::FlightState
DemandProcessor::getDemandedFlightModeMultiCopter()
{
  RxBase::SwitchPosition switchPosition = RxBase::getSwitch3Position(static_cast<uint32_t>(RxBase::ChannelName::MODE));

  switch (switchPosition)
  {
  case RxBase::SwitchPosition::SW_HIGH:
    return FlightState::SELF_LEVELLED;
    break;

  case RxBase::SwitchPosition::SW_MID:
    return FlightState::ACRO_TRAINER;
    break;

  default:  //Intentional fall through
  case RxBase::SwitchPosition::SW_LOW:
    return FlightState::RATE;
    break;
  }
}


/**
* @brief    Gets the heading hold demand.    
* @return   True when switch is high.
*/
bool
DemandProcessor::headingHoldActive()
{
  m_demand.headingHold = RxBase::isSwitchHigh(m_normalisedData.ch[static_cast<uint32_t>(RxBase::ChannelName::AUX2)]);
  return m_demand.headingHold;
}


/**
* @brief    Gets the prop hanging mode demand.    
* @return   True when switch is high.
*/
bool
DemandProcessor::propHangActive()
{
  return m_demand.propHang;
}


/**
* @brief    Indicates the failsafe state.    
* @return   Returns true if Rx has entered failsafe
*/
bool
DemandProcessor::inFailsafeState()
{
  return m_normalisedData.failsafe;
}


/**
* @brief    Prints DemandProcessor data to console for debugging purposes.
*/
void
DemandProcessor::printData()
{
  Serial.print("pitch: ");
  Serial.print(m_demand.pitch);
  Serial.print("\t roll: ");
  Serial.print(m_demand.roll);
  Serial.print("\t yaw: ");
  Serial.print(m_demand.yaw);
  Serial.print("\t throttle: ");
  Serial.print(m_demand.throttle);
  Serial.print("\t flaps: ");
  Serial.print(m_demand.flaps);
  Serial.print("\t armed: ");
  Serial.print(m_demand.armed);
  Serial.print("\t mode: ");
  Serial.print(static_cast<uint32_t>(getDemandedFlightModeFixedWing()));
  if constexpr(Config::USE_HEADING_HOLD)
  {
    Serial.print("\t Heading: ");
    Serial.print(m_demand.headingHold);
  }
  if constexpr(Config::USE_PROP_HANG_MODE)
  {
    Serial.print("\t PropHang: ");
    Serial.println(m_demand.propHang);
  }
  Serial.println();
}
