/* 
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
* @file   IBus.hpp
* @brief  This class handles communications with an IBus RC receiver.
*/

#pragma once
 
#include <inttypes.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "Timer.hpp"
#include "Config.hpp"
#include "RxBase.hpp"
 

/**
 * @class IBus
 * @brief IBus protocol decoder implementation derived from RxBase.
 */
class IBus : public RxBase
{
public:
   static constexpr uint32_t MIN_IBUS_US           = 1000;
   static constexpr uint32_t MID_IBUS_US           = 1500;
   static constexpr uint32_t MAX_IBUS_US           = 2000;
 
   /**
   * @brief Channel mapping for IBus protocol.
   * Maps standard channel names to physical IBus channel positions.
   */
   static constexpr uint32_t CHANNEL_MAP[18] =
   {
      0U,  // THROTTLE
      1U,  // ROLL
      2U,  // PITCH
      3U,  // YAW
      4U,  // AUX1
      5U,  // AUX2
      6U,  // AUX3
      7U,  // AUX4
      8U,  // AUX5
      9U,  // AUX6
      10U, // AUX7
      11U, // AUX8
      12U, // AUX9
      13U, // AUX10
      14U, // AUX11
      15U, // AUX12
      16U, // AUX13
      17U, // AUX14
   };

   static_assert((sizeof(CHANNEL_MAP) / sizeof(CHANNEL_MAP[0])) ==
     static_cast<uint32_t>(RcChannelName::COUNT),
     "Configuration Error: CHANNEL_MAP size must match RcChannelName::COUNT.");
 
   /**
   * @brief Constructor for IBus receiver.
   * @param uart Pointer to hardware serial port.
   * @param rxPin RX pin number.
   * @param txPin TX pin number.
   */
   IBus(HardwareSerial *uart, uint8_t rxPin, uint8_t txPin);

   /**
   * @brief Get new data from IBus receiver.
   * @return true when new data is available, false otherwise.
   */
   bool getDemands() override;
 
   /**
   * @brief Print receiver data to console for debugging.
   */
   void printData();
 
  /**
     * @brief Get receiver data packet.
     * @return RxPacket containing failsafe status, communication status, and normalised channel data.
     */
  const RxPacket getData() override;
 
  /**
     * @brief Check if communications have been lost.
     * @return true if communications lost, false otherwise.
     */
  const bool hasLostCommunications() override;
 
   /**
   * @brief Get channel index from channel name.
   * @param name The channel name enum.
   * @return Channel index (0-based).
   */
   uint32_t getChannelIndex(RcChannelName name) const
   {
      return CHANNEL_MAP[static_cast<uint32_t>(name)];
   }
 
private:
   //IBus specifications
   static constexpr uint32_t IBUS_BAUD          = 115200U;
   static constexpr uint32_t PAYLOAD_LEN        = 28U;
   static constexpr uint32_t HEADER_LEN         = 2U;
   static constexpr uint32_t CHECKSUM_LEN       = 2U;
   static constexpr uint32_t FRAME_LEN          = HEADER_LEN + PAYLOAD_LEN + CHECKSUM_LEN;
   static constexpr uint32_t CHECKSUM_BYTE_LEN  = HEADER_LEN + PAYLOAD_LEN;
   static constexpr uint32_t NUM_IBUS_CH        = 14U;
   //IBus protocol headers
   static constexpr uint32_t HEADER             = 0x20U;
   static constexpr uint32_t COMMAND            = 0x40U;
   //IBus checksum initial value
   static constexpr uint32_t CHECKSUM_START     = 0xFFFFU;
   //Internal usage
   static constexpr uint64_t COMMS_TIME_OUT_PERIOD = 100U;
   static constexpr uint32_t MAX_BYTES_PER_LOOP    = 6U;
 
   HardwareSerial *m_uart;
   uint32_t m_count = 0U;
   uint32_t m_prevByte = HEADER;
   uint32_t m_buff[FRAME_LEN] = {};
   bool m_lostFrame = false;

   //Objects
   CTimer lossOfCommsTimer = CTimer(0);
};
