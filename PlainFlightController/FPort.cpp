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
* @file   FPort.cpp
* @brief  This class handles communications with an F.Port RC receiver.
*/

#include "FPort.hpp"
#include "Utilities.hpp"


/**
* @brief    FPort constructor.
* @param    *uart  - Pointer to hardware serial port.
* @param    rxPin  - RX pin number.
* @param    txPin  - TX pin number (-1 disables TX, receive-only).
*/
FPort::FPort(HardwareSerial *uart, uint8_t rxPin, int8_t txPin)
{
  m_uart = uart;
  // F.Port: 115200 8N1, non-inverted TTL (invert=false).
  m_uart->begin(FPORT_BAUD, SERIAL_8N1, rxPin, txPin, false);
  m_uart->flush();
}


/**
* @brief    Decodes the F.Port serial stream into usable channel and flag data.
*
* F.Port frames are delimited by 0x7E markers. The markers themselves are
* NOT stored in m_buff; only the bytes in between are accumulated.
* Escape sequences (0x7D followed by byte XOR 0x20) are unstuffed before storage.
*
* When a closing 0x7E is received the accumulated buffer is passed to
* decodeControlFrame(). Non-control frames (e.g. downlink telemetry) are ignored.
*
* The full UART buffer is drained each call to match the timing requirements
* of F.Port at 115200 baud.
*
* @return   Returns true when a new valid control frame has been decoded.
*/
bool
FPort::getDemands()
{
  bool newData = false;

  uint32_t rxCount = 0U;
  while (m_uart->available() && (rxCount < MAX_BYTES_PER_LOOP))
  {
    uint8_t currentByte = static_cast<uint8_t>(m_uart->read());
    rxCount++;

    if (currentByte == FRAME_MARKER)
    {
      if (m_inFrame && (m_count > 0U))
      {
        // Closing marker: attempt to decode whatever is buffered.
        if (decodeControlFrame(m_count))
        {
          newData = true;
        }
      }
      // This 0x7E is simultaneously end of a previous frame and start of the next.
      m_inFrame = true;
      m_count   = 0U;
      m_escape  = false;
      continue;
    }

    // Ignore bytes received before the first frame marker.
    if (!m_inFrame)
    {
      continue;
    }

    // Handle byte-stuffing escape sequence.
    if (currentByte == ESCAPE_BYTE)
    {
      m_escape = true;
      continue;
    }

    if (m_escape)
    {
      currentByte ^= ESCAPE_XOR;
      m_escape = false;
    }

    // Guard against buffer overrun; discard frame if it exceeds expected size.
    if (m_count < BUFFER_SIZE)
    {
      m_buff[m_count] = currentByte;
      m_count++;
    }
    else
    {
      m_inFrame = false;
      m_count   = 0U;
    }
  }

  if (CTimer::State::EXPIRED == lossOfCommsTimer.getState())
  {
    m_rxData.lostComms = true;
  }

  return newData;
}


/**
* @brief    Validates the F.Port CRC of the buffered frame.
*
* End-around carry sum of bytes [0..(count-2)], when added to the CRC byte
* m_buff[count-1], must equal 0xFF.
*
* @param    count - Number of bytes in m_buff including the trailing CRC byte.
* @return   true if CRC is valid.
*/
bool
FPort::isCrcValid(uint32_t count) const
{
  uint16_t sum = 0U;

  for (uint32_t i = 0U; i < (count - 1U); i++)
  {
    sum += m_buff[i];
    // End-around carry: fold carry bit back into the low byte.
    if (sum > 0xFFU)
    {
      sum = (sum & 0xFFU) + 1U;
    }
  }

  return (static_cast<uint8_t>(sum + m_buff[count - 1U]) == 0xFFU);
}


/**
* @brief    Decodes a validated control frame from the buffer into channel data.
*
* Buffer layout (0x7E markers excluded):
*   [0]      LEN   (0x19)
*   [1]      TYPE  (0x00 = control)
*   [2..23]  22 bytes: 16-channel SBUS-identical bit-packed data
*   [24]     flags byte (LOST_FRAME_MASK = 0x04, FAILSAFE_MASK = 0x08)
*   [25]     RSSI byte
*   [26]     CRC
*
* @param    count - Number of bytes in m_buff.
* @return   true if the frame is a valid control frame with correct CRC.
*/
bool
FPort::decodeControlFrame(uint32_t count)
{
  if (count != BUFFER_SIZE)
  {
    return false;
  }

  if ((m_buff[IDX_LEN] != CTRL_LEN) || (m_buff[IDX_TYPE] != TYPE_CONTROL))
  {
    return false;
  }

  if (!isCrcValid(count))
  {
    return false;
  }

  // Channel payload pointer - bit-packing identical to SBUS.
  const uint8_t *p = &m_buff[IDX_PAYLOAD];

  uint32_t rawChannels[NUM_FPORT_CH];

  rawChannels[0]  = (uint32_t)( p[0]          | ((p[1]  << 8U)  & 0x07FFU));
  rawChannels[1]  = (uint32_t)((p[1]  >> 3U)  | ((p[2]  << 5U)  & 0x07FFU));
  rawChannels[2]  = (uint32_t)((p[2]  >> 6U)  |  (p[3]  << 2U)  | ((p[4]  << 10U) & 0x07FFU));
  rawChannels[3]  = (uint32_t)((p[4]  >> 1U)  | ((p[5]  << 7U)  & 0x07FFU));
  rawChannels[4]  = (uint32_t)((p[5]  >> 4U)  | ((p[6]  << 4U)  & 0x07FFU));
  rawChannels[5]  = (uint32_t)((p[6]  >> 7U)  |  (p[7]  << 1U)  | ((p[8]  << 9U)  & 0x07FFU));
  rawChannels[6]  = (uint32_t)((p[8]  >> 2U)  | ((p[9]  << 6U)  & 0x07FFU));
  rawChannels[7]  = (uint32_t)((p[9]  >> 5U)  | ((p[10] << 3U)  & 0x07FFU));

  if constexpr(Config::USE_PROP_HANG_MODE)
  {
    rawChannels[8]  = (uint32_t)( p[11]         | ((p[12] << 8U)  & 0x07FFU));
  }
  else
  {
    if constexpr(USE_ALL_18_CHANNELS)  //Speeds up main loop by not processing all 18 channels
    {
      static_assert(!USE_ALL_18_CHANNELS || (MAX_RX_CHANNELS >= 18), "MAX_RC_CHANNELS must be at least 18 when USE_ALL_18_CHANNELS is true");
      rawChannels[8]  = (uint32_t)( p[11]         | ((p[12] << 8U)  & 0x07FFU));
      rawChannels[9]  = (uint32_t)((p[12] >> 3U)  | ((p[13] << 5U)  & 0x07FFU));
      rawChannels[10] = (uint32_t)((p[13] >> 6U)  |  (p[14] << 2U)  | ((p[15] << 10U) & 0x07FFU));
      rawChannels[11] = (uint32_t)((p[15] >> 1U)  | ((p[16] << 7U)  & 0x07FFU));
      rawChannels[12] = (uint32_t)((p[16] >> 4U)  | ((p[17] << 4U)  & 0x07FFU));
      rawChannels[13] = (uint32_t)((p[17] >> 7U)  |  (p[18] << 1U)  | ((p[19] << 9U)  & 0x07FFU));
      rawChannels[14] = (uint32_t)((p[19] >> 2U)  | ((p[20] << 6U)  & 0x07FFU));
      rawChannels[15] = (uint32_t)((p[20] >> 5U)  | ((p[21] << 3U)  & 0x07FFU));
    }
  }

  // Normalise channels immediately after decoding.
  uint32_t numChannels = 8U;

  if constexpr(Config::USE_PROP_HANG_MODE)
  {
    numChannels = 9U;
  }
  else if constexpr (USE_ALL_18_CHANNELS)
  {
    numChannels = 18U;
  }

  for (uint32_t i = 0U; i < numChannels; i++)
  {
    m_rxData.ch[i] = map32(
      constrain(static_cast<int32_t>(rawChannels[i]),
                static_cast<int32_t>(MIN_FPORT_US),
                static_cast<int32_t>(MAX_FPORT_US)),
      static_cast<int32_t>(MIN_FPORT_US),
      static_cast<int32_t>(MAX_FPORT_US),
      MIN_NORMALISED,
      MAX_NORMALISED);
  }

  // Decode flags byte - same bit positions as SBUS.
  m_lostFrame       = (m_buff[IDX_FLAGS] & LOST_FRAME_MASK) ? true : false;
  m_rxData.failsafe = (m_buff[IDX_FLAGS] & FAILSAFE_MASK)   ? true : false;

  // Reset communications timer.
  lossOfCommsTimer.set(COMMS_TIME_OUT_PERIOD);
  m_rxData.lostComms = false;

  if constexpr(Config::DEBUG_RX)
  {
    printData();
  }

  return true;
}


/**
* @brief    Returns the most recent receiver data.
* @return   RxPacket containing failsafe, communication status, and normalised channel data.
*/
const RxBase::RxPacket
FPort::getData() const
{
  return m_rxData;
}


/**
* @brief    Check if communications have been lost.
* @return   true when F.Port comms has been lost, false otherwise.
*/
const bool
FPort::hasLostCommunications() const
{
  return m_rxData.lostComms;
}


/**
* @brief    Prints F.Port data to console for debugging.
*/
void
FPort::printData(void)
{
  static uint64_t lastPrintTime = 0U;
  const uint64_t now   = esp_timer_get_time();
  const uint64_t delta = now - lastPrintTime;
  lastPrintTime = now;

  // Display update time delta in microseconds.
  const uint32_t hz = static_cast<uint32_t>((delta > 0U) ? (1000000U / delta) : 0U);
  Serial.print("Hz=");
  Serial.print(hz, 1);
  Serial.print("\t");

  const uint32_t channels = (Config::USE_PROP_HANG_MODE) ? 9U : 8U;

  // Display the received normalised data.
  for (uint32_t i = 0U; i < channels; i++)
  {
    Serial.print(m_rxData.ch[i]);
    Serial.print("\t");
  }

  // Display lost frame and failsafe data.
  Serial.print(m_lostFrame);
  Serial.print("\t");
  Serial.print(m_rxData.failsafe);
  Serial.print("\t");
  Serial.println(m_rxData.lostComms);
}
