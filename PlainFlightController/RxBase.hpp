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
* @file   RxBase.hpp
* @brief  Base class for RC receiver implementations providing common interface and normalization.
*/

#pragma once

#include <inttypes.h>

/**
 * @class RxBase
 * @brief Base class for all receiver types, providing common constants, channel mapping, and data structures.
 */
class RxBase
{
public:
  static constexpr uint32_t MAX_RX_CHANNELS = 16U;
  static constexpr int32_t MIN_NORMALISED = -1024;
  static constexpr int32_t MID_NORMALISED = 0;
  static constexpr int32_t MAX_NORMALISED = 1024;
  static constexpr int32_t SWITCH_HIGH_NORM = 512;
  static constexpr int32_t SWITCH_LOW_NORM = -512;
  static constexpr int32_t LOW_THROTTLE_NORM = -922;  // Changed to be 5% of range - post test fix

  /**
     * @enum ReceiverType
     * @brief Class of receiver.  Add to this list when new receiver protocols defined
     */
  enum class ReceiverType
  {
    SBUS,
    CRSF
  };

  /**
     * @enum ChannelName
     * @brief Standard channel names for RC control mapping.
     * Other channels must be referred to by number
     */
  enum class ChannelName : uint32_t
  {
    THROTTLE = 0U,
    ROLL,
    PITCH,
    YAW,
    ARM,
    MODE,
    AUX1,
    AUX2,
    AUX3
  };

  /**
     * @enum SwitchPosition
     * @brief Enumeration for switch states.
     */
  enum class SwitchPosition : uint32_t
  {
    SW_LOW = 0U,
    SW_MID,
    SW_HIGH
  };

  /**
     * @struct RxPacket
     * @brief Standardized receiver data packet for all receiver types.
     */
  struct RxPacket
  {
    bool failsafe;
    bool lostComms;
    int32_t ch[MAX_RX_CHANNELS];  // Normalised data
  };

  /**
     * @brief Virtual destructor for proper cleanup of derived classes.
     */
  virtual ~RxBase() = default;

  /**
     * @brief Get receiver data packet.
     * @return RxPacket containing failsafe status, communication status, and normalised channel data.
     */
  virtual const RxPacket
  getData() const = 0;

  /**
     * @brief Check if communications have been lost.
     * @return true if communications lost, false otherwise.
     */
  virtual const bool
  hasLostCommunications() const = 0;

  /**
     * @brief Get new data from receiver.
     * @return true when new data is available, false otherwise.
     */
  virtual bool
  getDemands() = 0;

  /**
     * @brief Get channel index from channel name.
     * @param name The channel name enum.
     * @return Channel index (0-based).
     */
  virtual uint32_t
  getChannelIndex(ChannelName name) const = 0;

  /**
     * @brief Helper function to reduce verbosity.
     * @param name The channel name enum (from RxBase).
     * @return Channel index (0-based).
     */
  int32_t getChannel(const RxPacket& data, ChannelName name) const
  {
    return data.ch[getChannelIndex(name)];
  }

  /**
     * @brief Check if a channel is in the high switch position.
     * @param channelData The normalised channel data to check.
     * @return true if channel value is above high threshold, false otherwise.
     */
  static bool
  isSwitchHigh(const int32_t channelData);

  /**
     * @brief Check if a channel is in the low switch position.
     * @param channelData The normalised channel data to check.
     * @return true if channel value is below low threshold, false otherwise.
     */
  static bool
  isSwitchLow(const int32_t channelData);

  /**
     * @brief Get the 3-position switch state for normalised channel data.
     * @param channelData The normalised channel data to check.
     * @return SwitchPosition enum (LOW, MID, or HIGH).
     */
  static SwitchPosition
  getSwitch3Position(const int32_t channelData);

  /**
     * @brief Get the 2-position switch state for normalised channel data.
     * @param channelData The normalised channel data to check.
     * @return SwitchPosition enum (LOW or HIGH).
     */
  static SwitchPosition
  getSwitch2Position(const int32_t channelData);

protected:
  RxPacket m_rxData = {true, true, {0}};

  /**
     * @brief Default constructor (protected to prevent direct instantiation).
     */
  RxBase() = default;
};
