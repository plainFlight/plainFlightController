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
* @file   Crsf.cpp
* @brief  CRSF/ELRS receiver implementation.
*
* Changes from the original Crsf.cpp
* ------------------------------------
* - crc8_dvb_s2() and calculateCrc() removed; replaced by calls to the
*   equivalent static methods in CrsfCodec.
*
* - All wire-format constants (frame offsets, device addresses, frame type
*   bytes, channel ranges) replaced with CrsfCodec:: qualified names; only
*   transport and application-policy constants remain local to this class.
*
* - parseRcChannels(): the inline 11-bit channel unpacking replaced by a
*   single call to CrsfCodec::unpackChannels().  The frame-length guard was
*   also corrected (see the function's implementation comment for details).
*
* - parseLinkStatistics(): parameter renamed payloadLength → frameLength for
*   accuracy (the value is the CRSF frame-length field, not a raw payload byte
*   count).  Guard updated to use CrsfCodec::LINK_STATS_MIN_FRAME_LENGTH.
*/

#include "Crsf.hpp"
#include "InternalConfig.hpp"
#include "Utilities.hpp"


/**
* @brief    CRSF constructor.
* @details  Initialises the UART at CRSF_BAUD (420000 for ELRS) and flushes any
*           stale bytes so that getDemands() begins with a clean stream.
* @param    uart   Pointer to the hardware serial port to use.
* @param    rxPin  GPIO pin number for the UART RX line.
* @param    txPin  GPIO pin number for the UART TX line.
*/
Crsf::Crsf(HardwareSerial* const uart, const uint8_t rxPin, const uint8_t txPin)
{
  m_uart = uart;
  m_uart->begin(CRSF_BAUD, SERIAL_8N1, rxPin, txPin, false);
  m_uart->flush();
}


/**
* @brief    Decodes the CRSF serial stream into usable channel and flag data.
* @details  Implements a three-state byte-stream parser.  Reads at most
*           MAX_BYTES_PER_LOOP bytes per call so the main loop is never blocked
*           for longer than a bounded time (~190 µs at 420000 baud).
*
*           State 0: hunt for a valid sync byte (ADDR_FLIGHT_CONTROLLER or
*                    ADDR_RECEIVER — either address may appear on a shared bus).
*           State 1: read and range-check the frame-length byte.
*           State 2+: accumulate bytes; when the expected number have arrived,
*                    validate the CRC and dispatch to the appropriate parser.
*
* @return   true when a new, valid RC channels frame has been decoded and
*           m_rxData updated; false on all other calls.
*/
bool
Crsf::getDemands()
{
  uint32_t currentByte;
  uint32_t rxCount = 0U;

  while (m_uart->available() && (rxCount < MAX_BYTES_PER_LOOP))
  {
    currentByte = m_uart->read();
    rxCount++;

    // State 0: hunt for a valid sync byte.
    // Both the FC address (0xC8) and the receiver address (0xEC) are accepted
    // because either may appear as the sync byte on a shared bus.
    if (0U == m_bufferIndex)
    {
      if ((currentByte == static_cast<uint32_t>(CrsfCodec::ADDR_FLIGHT_CONTROLLER)) ||
          (currentByte == static_cast<uint32_t>(CrsfCodec::ADDR_RECEIVER)))
      {
        m_buffer[m_bufferIndex] = static_cast<uint8_t>(currentByte);
        m_bufferIndex++;
      }
    }
    // State 1: read the frame-length byte.
    // Valid range: 2 to (MAX_FRAME_SIZE - 2).  The lower bound (2) represents
    // the minimum meaningful frame (type + CRC, no payload).  The upper bound
    // leaves room for the sync and length bytes themselves.
    else if (1U == m_bufferIndex)
    {
      if ((currentByte >= 2U) && (currentByte <= (CrsfCodec::MAX_FRAME_SIZE - 2U)))
      {
        m_buffer[m_bufferIndex] = static_cast<uint8_t>(currentByte);
        m_bufferIndex++;
      }
      else
      {
        // Length out of range — discard and restart hunt
        m_bufferIndex = 0U;
      }
    }
    // State 2+: accumulate bytes until the full frame is buffered.
    else
    {
      m_buffer[m_bufferIndex] = static_cast<uint8_t>(currentByte);
      m_bufferIndex++;

      // Total frame buffer occupancy = sync(1) + length(1) + frame_length bytes.
      // Wait until all of them have arrived before attempting to validate.
      const uint32_t frameLength = static_cast<uint32_t>(m_buffer[CrsfCodec::LENGTH_OFFSET]);
      if (static_cast<uint32_t>(m_bufferIndex) >= (frameLength + 2U))
      {
        // CRC covers the type byte and all payload bytes; it excludes sync and length.
        // frame_length = type(1) + payload bytes + CRC(1), so (frame_length - 1)
        // is the number of bytes to pass to calculateCrc (type + payload).
        const uint32_t crcByteIndex  = frameLength + 1U;
        const uint8_t  calculatedCrc = CrsfCodec::calculateCrc(
                                         &m_buffer[CrsfCodec::TYPE_OFFSET],
                                         static_cast<uint8_t>(frameLength - 1U));

        if (calculatedCrc == m_buffer[crcByteIndex])
        {
          // CRC valid — identify frame type and dispatch
          const uint32_t  frameType = static_cast<uint32_t>(m_buffer[CrsfCodec::TYPE_OFFSET]);
          const uint8_t*  payload   = &m_buffer[CrsfCodec::PAYLOAD_OFFSET];

          if (static_cast<uint32_t>(CrsfCodec::FRAMETYPE_RC_CHANNELS) == frameType)
          {
            parseRcChannels(payload, static_cast<uint8_t>(frameLength));

            // Valid RC frame received — reset the loss-of-comms timer
            lossOfCommsTimer.set(COMMS_TIME_OUT_PERIOD);
            m_rxData.lostComms = false;

            if constexpr(InternalConfig::DEBUG_RX)
            {
              printData();
            }

            m_bufferIndex = 0U;
            return true;
          }
          else if (static_cast<uint32_t>(CrsfCodec::FRAMETYPE_LINK_STATS) == frameType)
          {
            parseLinkStatistics(payload, static_cast<uint8_t>(frameLength));
          }
          else
          {
            // Unknown or unhandled frame type — discard silently
          }
        }

        // Frame complete (valid or invalid CRC) — reset for next frame
        m_bufferIndex = 0U;
      }

      // Safety guard: if somehow bufferIndex reaches the buffer limit without
      // a complete frame being detected, reset rather than overflow the buffer.
      if (static_cast<uint32_t>(m_bufferIndex) >= CrsfCodec::MAX_FRAME_SIZE)
      {
        m_bufferIndex = 0U;
      }
    }
  }

  if (CTimer::State::EXPIRED == lossOfCommsTimer.getState())
  {
    m_rxData.lostComms = true;
  }

  return false;
}


/**
* @brief    Parse a CRSF RC channels payload (frame type 0x16) and update m_rxData.
* @details  Guards against short frames, delegates 11-bit channel unpacking to
*           CrsfCodec::unpackChannels(), normalises the raw values into the
*           application range, and updates the multi-level failsafe flag.
*
*           Frame-length guard correction
*           ------------------------------
*           The original code checked `frameLength < RC_PAYLOAD_BYTES (22)`.
*           However, frameLength is the CRSF frame-length field, which equals
*           type(1) + payload_bytes + CRC(1).  For a valid RC frame this value
*           is 24, not 22.  A frame with frameLength = 22 would only carry 20
*           bytes of payload — not enough for 16 channels.  The guard is now
*           `frameLength < (RC_PAYLOAD_BYTES + 2U)` (i.e., < 24) which is the
*           correct minimum for a complete RC channels frame.
*
* @param    payload      Pointer to the first payload byte in the frame buffer.
* @param    frameLength  The CRSF frame-length field value (type + payload + CRC).
*/
void
Crsf::parseRcChannels(const uint8_t* payload, uint8_t frameLength)
{
  // Minimum frame-length field value for a valid RC channels frame:
  // type(1) + 22 payload bytes + CRC(1) = 24.
  if (frameLength < (CrsfCodec::RC_PAYLOAD_BYTES + 2U))
  {
    return;
  }

  uint32_t rawChannels[CrsfCodec::NUM_CHANNELS];

  // Unpack 16 x 11-bit channel values from the 22-byte little-endian bitstream.
  // All bit manipulation is encapsulated in CrsfCodec::unpackChannels(); see
  // CrsfCodec.cpp for a detailed explanation of the packing scheme.
  CrsfCodec::unpackChannels(payload, rawChannels);

  // Normalise each raw value from the CRSF range [172..1811] into the
  // application range [MIN_NORMALISED..MAX_NORMALISED] (-1024..+1024).
  for (uint32_t i = 0U; i < CrsfCodec::NUM_CHANNELS; i++)
  {
    m_rxData.ch[i] = map32(rawChannels[i],
                           CrsfCodec::MIN_CHANNEL_VALUE,
                           CrsfCodec::MAX_CHANNEL_VALUE,
                           MIN_NORMALISED,
                           MAX_NORMALISED);
  }

  // Multi-level failsafe: assert if communications are lost OR if link quality
  // has dropped below the threshold even though frames are still arriving.
  m_rxData.failsafe = m_rxData.lostComms ||
                      (m_crsfLinkStats.linkQuality < FAILSAFE_LQ_THRESHOLD);
}


/**
* @brief    Parse a CRSF link statistics payload (frame type 0x14) and update
*           m_crsfLinkStats.
* @details  Reads the uplink RSSI, link quality, SNR, and TX power fields from
*           the payload at the byte offsets defined by the CRSF 0x14 frame spec.
*           The link quality value is subsequently used by parseRcChannels() for
*           the secondary failsafe threshold check.
*
* @param    payload      Pointer to the first payload byte in the frame buffer.
* @param    frameLength  The CRSF frame-length field value (type + payload + CRC).
*/
void
Crsf::parseLinkStatistics(const uint8_t* payload, uint8_t frameLength)
{
  // Minimum frame-length field value for a valid link statistics frame:
  // type(1) + 10 payload bytes + CRC(1) = 12.
  if (frameLength < CrsfCodec::LINK_STATS_MIN_FRAME_LENGTH)
  {
    return;
  }

  // Byte offsets within the 0x14 payload (CRSF spec: Link Statistics)
  //   [0] up_rssi_ant1     — uplink RSSI antenna 1 (dBm * -1, so negate to get dBm)
  //   [1] up_rssi_ant2     — uplink RSSI antenna 2 (unused here)
  //   [2] up_link_quality  — uplink packet success rate (%)
  //   [3] up_snr           — uplink SNR (dB, signed)
  //   [4] active_antenna   — unused here
  //   [5] rf_profile       — unused here
  //   [6] up_rf_power      — transmitter power index
  m_crsfLinkStats.rssiDbm     = static_cast<int8_t>(payload[0]);
  m_crsfLinkStats.linkQuality = static_cast<uint8_t>(payload[2]);
  m_crsfLinkStats.snrDb       = static_cast<int8_t>(payload[3]);
  m_crsfLinkStats.txPower     = static_cast<uint8_t>(payload[6]);
}


/**
* @brief    Returns the most recent receiver data packet.
* @return   RxPacket containing failsafe flag, lostComms flag, and normalised
*           channel data in range [MIN_NORMALISED .. MAX_NORMALISED].
*/
const RxBase::RxPacket
Crsf::getData() const
{
  return m_rxData;
}


/**
* @brief    Report whether CRSF communications have been lost.
* @return   true if no valid RC channels frame has been received within
*           COMMS_TIME_OUT_PERIOD milliseconds.
*/
const bool
Crsf::hasLostCommunications() const
{
  return m_rxData.lostComms;
}


/**
* @brief    Returns the most recently received CRSF link quality statistics.
* @return   CrsfLinkStats populated from the last valid 0x14 frame.
*/
Crsf::CrsfLinkStats
Crsf::getLinkStats() const
{
  return m_crsfLinkStats;
}


/**
* @brief    Prints channel data and status flags to Serial for debugging.
* @details  Rate-limited by comparing against the last print time so that the
*           output remains readable at high RC frame rates.  The update rate in
*           Hz is derived from the inter-call delta and printed first.
*           Only active when InternalConfig::DEBUG_RX is true (checked by the
*           caller via if constexpr in getDemands()).
*/
void
Crsf::printData(void)
{
  static uint64_t lastPrintTime = 0U;   // Static is safe: only one Crsf instance exists.
  const uint64_t  now   = esp_timer_get_time();
  const uint64_t  delta = now - lastPrintTime;
  lastPrintTime = now;

  const uint32_t hz = static_cast<uint32_t>((delta > 0U) ? (1000000U / delta) : 0U);
  Serial.print("Hz=");
  Serial.print(hz, 1);
  Serial.print("\t");

  for (uint32_t i = 0U; i < CrsfCodec::NUM_CHANNELS; i++)
  {
    Serial.print(m_rxData.ch[i]);
    Serial.print("\t");
  }

  Serial.print(m_rxData.failsafe);
  Serial.print("\t");
  Serial.println(m_rxData.lostComms);
}