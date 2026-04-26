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
* @file   FPort.hpp
* @brief  This class handles communications with an F.Port RC receiver.
*/

#pragma once

#include <inttypes.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "Timer.hpp"
#include "Config.hpp"
#include "RxBase.hpp"


/**
 * @class FPort
 * @brief F.Port protocol decoder implementation derived from RxBase.
 *
 * F.Port protocol documentation:
 * https://github.com/betaflight/betaflight/files/1491056/F.Port.protocol.betaFlight.V2.1.2017.11.21.pdf
 * 
 * Decodes the F.Port V2.1 control frame (type 0x00).
 * Channel payload is identical to SBUS; 115200 8N1 non-inverted.
 * Byte stuffing (0x7D escape sequences) is handled on receive.
 * Telemetry downlink frames (type 0x01) are detected then discarded.
 *
 * Buffer layout (0x7E frame markers excluded, bytes stored after unstuffing):
 *   [0]      LEN   (0x19 for a control frame)
 *   [1]      TYPE  (0x00 = control, 0x01 = downlink)
 *   [2..23]  22 bytes: 16-channel SBUS-identical bit-packed data
 *   [24]     flags byte (lost frame, failsafe bits)
 *   [25]     RSSI byte
 *   [26]     CRC
 * Total: 27 bytes between markers.
 */
class FPort : public RxBase
{
public:
  static constexpr uint32_t MIN_FPORT_US    = 172U;
  static constexpr uint32_t MID_FPORT_US    = 991U;
  static constexpr uint32_t MAX_FPORT_US    = 1810U;
  static constexpr bool USE_ALL_18_CHANNELS = false;
  

  /**
   * @brief Channel mapping for F.Port protocol.
   * Maps standard channel names to physical F.Port channel positions.
   */
  static constexpr uint32_t CHANNEL_MAP[9] =
      {
          2U,  // THROTTLE
          0U,  // ROLL
          1U,  // PITCH
          3U,  // YAW
          4U,  // ARM
          5U,  // MODE
          6U,  // AUX1
          7U,  // AUX2
          8U   // AUX3
      };

  /**
   * @brief Constructor for F.Port receiver.
   * @param uart  Pointer to hardware serial port.
   * @param rxPin RX pin number.
   * @param txPin TX pin number (default -1, disabled for receive-only use).
   */
  FPort(HardwareSerial *uart, uint8_t rxPin, int8_t txPin = -1);

  /**
   * @brief Get new data from F.Port receiver.
   * @return true when a new valid control frame has been decoded.
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
  uint32_t getChannelIndex(ChannelName name) const
  {
    return CHANNEL_MAP[static_cast<uint32_t>(name)];
  }

private:
  static constexpr uint32_t FPORT_BAUD            = 115200U;

  // Buffer stores bytes between 0x7E markers (markers themselves excluded).
  // Verified from live capture: 27 bytes per control frame.
  static constexpr uint32_t BUFFER_SIZE           = 27U;
  static constexpr uint32_t NUM_FPORT_CH          = 16U;

  static constexpr uint8_t  FRAME_MARKER          = 0x7EU;
  static constexpr uint8_t  ESCAPE_BYTE           = 0x7DU;
  static constexpr uint8_t  ESCAPE_XOR            = 0x20U;

  // Indices into m_buff (0x7E markers excluded, verified from live capture).
  static constexpr uint32_t IDX_LEN               = 0U;   // LEN byte (0x19)
  static constexpr uint32_t IDX_TYPE              = 1U;   // TYPE byte
  static constexpr uint32_t IDX_PAYLOAD           = 2U;   // First SBUS channel byte

  static constexpr uint8_t  CTRL_LEN              = 0x19U;  // Expected LEN for a control frame
  static constexpr uint8_t  TYPE_CONTROL          = 0x00U;

  // buf[2..23] = 22 bytes of SBUS-packed channel data (16 channels x 11 bit)
  // buf[24]    = flags byte
  // buf[25]    = RSSI byte
  // buf[26]    = CRC byte
  static constexpr uint32_t IDX_FLAGS             = 24U;
  static constexpr uint32_t IDX_RSSI              = 25U;
  static constexpr uint32_t IDX_CRC               = 26U;

  static constexpr uint32_t LOST_FRAME_MASK       = 0x04U;
  static constexpr uint32_t FAILSAFE_MASK         = 0x08U;

  static constexpr uint64_t COMMS_TIME_OUT_PERIOD = 100U;
  static constexpr uint32_t MAX_BYTES_PER_LOOP    = 32U;

  HardwareSerial *m_uart;
  uint32_t m_count   = 0U;
  bool     m_inFrame = false;   // True once the opening 0x7E marker has been received
  bool     m_escape  = false;   // True when previous byte was ESCAPE_BYTE
  uint8_t  m_buff[BUFFER_SIZE] = {};
  bool     m_lostFrame = false;
  // Objects
  CTimer lossOfCommsTimer = CTimer(0);

  /**
   * @brief Validate F.Port CRC over the buffered frame content.
   * End-around carry sum of bytes [0..(count-2)], when added to m_buff[count-1], must equal 0xFF.
   * @param count Number of bytes in m_buff including the trailing CRC byte.
   * @return true if CRC is valid.
   */
  bool isCrcValid(uint32_t count) const;

  /**
   * @brief Decode a validated control frame buffer into channel data and flags.
   * @param count Number of bytes in m_buff.
   * @return true on successful decode.
   */
  bool decodeControlFrame(uint32_t count);
};
