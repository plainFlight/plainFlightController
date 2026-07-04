/* 
* Original file by D. Gamble (Github: Cyberslug)
*
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
* @file   ChannelValidation.hpp
* @brief  Compile-time checks confirming no two RC functions share a channel.
*/
#pragma once

#include "CommonTypes.hpp"
#include "Config.hpp"

/**
 * @namespace ChannelValidation
 */
namespace ChannelValidation
{
  static constexpr uint32_t toIndex(RcChannelName name)
  {
    return static_cast<uint32_t>(name);
  }

  // Derived directly from Config.hpp, matching the same technique used for
  // InternalConfig::NUMBER_PASS_THROUGH,  kept local to this file so it does
  // not need to include InternalConfig.hpp.
  static constexpr uint32_t PASS_THROUGH_COUNT =
    static_cast<uint32_t>(sizeof(Config::PASS_THROUGH_PINS) / sizeof(Config::PASS_THROUGH_PINS[0]));

  /**
  * @brief  Confirms no two assigned RC functions share the same channel.
  * @return true if any two assigned channels collide, false otherwise.
  */
  static constexpr bool channelIsDuplicated()
  {
    uint32_t used[8] = {};
    uint32_t count = 0U;

    const RcChannelName candidates[] =
    {
      Config::ARM_CHANNEL,
      Config::MODE_CHANNEL,
      Config::FLAPS_CHANNEL,
      Config::HEADING_HOLD_CHANNEL,
      Config::PROP_HANG_CHANNEL,
    };

    for (const RcChannelName candidate : candidates)
    {
      if (candidate != RcChannelName::NONE)
      {
        used[count++] = toIndex(candidate);
      }
    }
    for (uint32_t i = 0U; i < PASS_THROUGH_COUNT; ++i)
    {
      used[count++] = toIndex(Config::PASS_THROUGH_PINS[i].channel);
    }

    for (uint32_t i = 0U; i < count; ++i)
    {
      for (uint32_t j = (i + 1U); j < count; ++j)
      {
        if (used[i] == used[j])
        {
          return true;
        }
      }
    }
    return false;
  }
}