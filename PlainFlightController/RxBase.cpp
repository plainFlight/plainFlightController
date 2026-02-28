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
* @file   RxBase.cpp
* @brief  Base class implementation for RC receiver switch state checking.
*/

#include "RxBase.hpp"


/**
* @brief    Check if a channel is in the high switch position.
* @param    channelData - The normalised channel data to check.
* @return   true if channel value is above high threshold, false otherwise.
*/
bool
RxBase::isSwitchHigh(const int32_t channelData)
{
  bool result = false;

  if (channelData > SWITCH_HIGH_NORM)
  {
    result = true;
  }

  return result;
}


/**
* @brief    Check if a channel is in the low switch position.
* @param    channelData - The normalised channel data to check.
* @return   true if channel value is below low threshold, false otherwise.
*/
bool
RxBase::isSwitchLow(const int32_t channelData)
{
  bool result = false;

  if (channelData < SWITCH_LOW_NORM)
  {
    result = true;
  }

  return result;
}


/**
* @brief    Get the 3-position switch state for normalised channel data.
* @param    channelData - The normalised channel data to check.
* @return   SwitchPosition enum (LOW, MID, or HIGH).
*/
RxBase::SwitchPosition
RxBase::getSwitch3Position(const int32_t channelData)
{
  SwitchPosition position = SwitchPosition::SW_MID;

  if (channelData > SWITCH_HIGH_NORM)
  {
    position = SwitchPosition::SW_HIGH;
  }
  else if (channelData < SWITCH_LOW_NORM)
  {
    position = SwitchPosition::SW_LOW;
  }
  else
  {
    position = SwitchPosition::SW_MID;
  }

  return position;
}


/**
* @brief    Get the 2-position switch state for normalised channel data.
* @param    channelData - The normalised channel data to check.
* @return   SwitchPosition enum (LOW or HIGH).
*/
RxBase::SwitchPosition
RxBase::getSwitch2Position(const int32_t channelData)
{
  SwitchPosition position = SwitchPosition::SW_LOW;

  if (channelData > SWITCH_HIGH_NORM)
  {
    position = SwitchPosition::SW_HIGH;
  }
  else
  {
    position = SwitchPosition::SW_LOW;
  }

  return position;
}