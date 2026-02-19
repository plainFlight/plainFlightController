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
* @file   SBus.hpp
* @brief  This class handles communications with an SBus RC receiver.
*/

/*
* While very different, this module has been inspired by and gives credit to:
*
* Brian R Taylor
* brian.taylor@bolderflight.com
* Copyright (c) 2022 Bolder Flight Systems Inc
* https://github.com/bolderflight/sbus
*/
#pragma once

#include <inttypes.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "Timer.hpp"
#include "Config.hpp"
#include "RxBase.hpp"


/**
 * @class SBus
 * @brief SBus protocol decoder implementation derived from RxBase.
 */
class SBus : public RxBase
{
public:
  static constexpr uint32_t MIN_SBUS_US           = 172U;
  static constexpr uint32_t MID_SBUS_US           = 991U;
  static constexpr uint32_t MAX_SBUS_US           = 1810U;
  static constexpr bool USE_ALL_18_CHANNELS       = false;

  /**
     * @brief Channel mapping for SBus protocol.
     * Maps standard channel names to physical SBus channel positions.
     */
  static constexpr uint32_t CHANNEL_MAP[9] =
      {
          0U,  // THROTTLE
          1U,  // ROLL
          2U,  // PITCH
          3U,  // YAW
          4U,  // ARM
          5U,  // MODE
          6U,  // AUX1
          7U,  // AUX2
          8U  // AUX3
      };

  /**
     * @brief Constructor for SBus receiver.
     * @param uart Pointer to hardware serial port.
     * @param rxPin RX pin number.
     * @param txPin TX pin number.
     */
  SBus(HardwareSerial *uart, uint8_t rxPin, uint8_t txPin);

  /**
     * @brief Get new data from SBus receiver.
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
  const RxPacket getData() const override;

  /**
     * @brief Check if communications have been lost.
     * @return true if communications lost, false otherwise.
     */
  const bool hasLostCommunications() const override;

  /**
     * @brief Get channel index from channel name.
     * @param name The channel name enum.
     * @return Channel index (0-based).
     */
  constexpr uint32_t getChannelIndex(ChannelName name) const
  {
    return CHANNEL_MAP[static_cast<uint32_t>(name)];
  }

private:
  static constexpr uint32_t SBUS_BAUD       = 100000U;
  static constexpr uint32_t PAYLOAD_LEN     = 23U;
  static constexpr uint32_t HEADER_LEN      = 1U;
  static constexpr uint32_t FOOTER_LEN      = 1U;
  static constexpr uint32_t NUM_SBUS_CH     = 16U;
  static constexpr uint32_t HEADER          = 0x0FU;
  static constexpr uint32_t FOOTER          = 0x00U;
  static constexpr uint32_t FOOTER2         = 0x04U;
  static constexpr uint32_t CH17_MASK       = 0x01U;
  static constexpr uint32_t CH18_MASK       = 0x02U;
  static constexpr uint32_t LOST_FRAME_MASK = 0x04U;
  static constexpr uint32_t FAILSAFE_MASK   = 0x08U;
  static constexpr uint32_t BUFFER_SIZE     = 25U;
  static constexpr uint64_t COMMS_TIME_OUT_PERIOD = 100U;

  HardwareSerial *m_uart;
  uint32_t m_count = 0U;
  uint32_t m_prevByte = FOOTER;
  uint32_t m_buff[BUFFER_SIZE] = {};
  bool m_lostFrame = false;
  //Objects
  CTimer lossOfCommsTimer = CTimer(0);
};