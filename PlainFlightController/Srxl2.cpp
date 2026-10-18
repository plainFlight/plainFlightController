/*
* Original File Author: D. Gamble (Github: Cyberslug)
*
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
* @file   Srxl2.cpp
* @brief  Spektrum SRXL2 receiver implementation: RC channel data in, no
*         telemetry reply. See Srxl2.hpp for scope and notes.
*/

#include "Srxl2.hpp"
#include "InternalConfig.hpp"
#include "Utilities.hpp"


// =============================================================================
// Codec: CRC-16, frame validation, channel-mask unpacking, Handshake frame
// building. No Arduino dependency.
// =============================================================================

/**
* @brief   Process one byte through the CRC-16/XMODEM algorithm (polynomial 0x1021).
* @details MSB-first, no input/output reflection, no final XOR - the standard
*          XMODEM variant. All-uint32_t intermediate arithmetic avoids the
*          implicit promotion-to-signed-int issue that shifting sub-int types
*          can otherwise cause.
* @param   crc        Running CRC accumulator (0x0000 on first call).
* @param   byteValue  Byte to fold into the CRC.
* @return  Updated CRC-16 value.
*/
static uint16_t
crc16Step(const uint16_t crc, const uint8_t byteValue)
{
  uint32_t result = static_cast<uint32_t>(crc) ^ (static_cast<uint32_t>(byteValue) << 8U);

  for (uint32_t i = 0U; i < 8U; i++)
  {
    if ((result & 0x8000U) != 0U)
    {
      result = ((result << 1U) ^ 0x1021U) & 0xFFFFU;
    }
    else
    {
      result = (result << 1U) & 0xFFFFU;
    }
  }

  return static_cast<uint16_t>(result);
}


/**
* @brief   Calculate CRC-16/XMODEM over a data buffer.
* @param   data    Pointer to the first byte to include.
* @param   length  Number of bytes to process.
* @return  Final CRC-16 value.
*/
uint16_t
Srxl2::calculateCrc(const uint8_t* const data, const uint32_t length)
{
  uint16_t crc = 0U;

  for (uint32_t i = 0U; i < length; i++)
  {
    crc = crc16Step(crc, data[i]);
  }

  return crc;
}


/**
* @brief   Validate a complete frame buffer's trailing CRC-16.
* @details The CRC covers the whole frame excluding its own two trailing
*          bytes - that includes the sync byte and length byte. The received
*          CRC is big-endian on the wire.
* @param   buffer  Pointer to the frame buffer, starting at the sync byte.
* @param   length  Total frame length (buffer[LENGTH_OFFSET]).
* @return  true if length is plausible and the CRC matches.
*/
bool
Srxl2::isFrameCrcValid(const uint8_t* const buffer, const uint32_t length)
{
  bool valid = false;

  if (length >= MIN_FRAME_LENGTH)
  {
    const uint32_t crcCoveredLength = length - CRC_LENGTH;
    const uint16_t calculated       = calculateCrc(buffer, crcCoveredLength);
    const uint16_t received         = static_cast<uint16_t>(
                                         (static_cast<uint32_t>(buffer[crcCoveredLength]) << 8U)
                                         | static_cast<uint32_t>(buffer[crcCoveredLength + 1U]));
    valid = (calculated == received);
  }

  return valid;
}


/**
* @brief   Count how many of the 32 possible channel-mask bits are set.
* @param   mask  A Control Data packet's channel mask field.
* @return  Number of active (present) channels.
*/
uint32_t
Srxl2::countActiveChannels(const uint32_t mask)
{
  uint32_t count = 0U;

  for (uint32_t bit = 0U; bit < MAX_SRXL_CHANNELS; bit++)
  {
    if ((mask & (1UL << bit)) != 0U)
    {
      count++;
    }
  }

  return count;
}


/**
* @brief   Read a little-endian 16-bit value from a buffer.
*/
uint32_t
Srxl2::readLE16(const uint8_t* const p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8U);
}


/**
* @brief   Read a little-endian 32-bit value from a buffer.
*/
uint32_t
Srxl2::readLE32(const uint8_t* const p)
{
  return static_cast<uint32_t>(p[0])
       | (static_cast<uint32_t>(p[1]) << 8U)
       | (static_cast<uint32_t>(p[2]) << 16U)
       | (static_cast<uint32_t>(p[3]) << 24U);
}


/**
* @brief   Write a 32-bit value into a buffer, little-endian.
*/
void
Srxl2::writeLE32(uint8_t* const p, const uint32_t value)
{
  p[0] = static_cast<uint8_t>(value & 0xFFU);
  p[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  p[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  p[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}


/**
* @brief   Unpack raw channel values from a Control Data packet's channel-value region.
* @details Wire values are little-endian uint16, one per set mask bit, packed
*          contiguously in ascending bit-index order (no gaps for absent
*          channels). Only entries whose mask bit is set are written into
*          rawValues; other slots are left untouched.
* @param   valuesStart  Pointer to the first channel-value byte.
* @param   mask         The frame's channel mask field.
* @param   rawValues    Output: raw values indexed by channel/bit number (0-31).
* @return  Number of channels unpacked (== countActiveChannels(mask)).
*/
uint32_t
Srxl2::unpackChannelValues(const uint8_t* const valuesStart, const uint32_t mask,
                            uint32_t (&rawValues)[MAX_SRXL_CHANNELS])
{
  uint32_t channelCount = 0U;
  uint32_t byteOffset   = 0U;

  for (uint32_t bit = 0U; bit < MAX_SRXL_CHANNELS; bit++)
  {
    if ((mask & (1UL << bit)) != 0U)
    {
      rawValues[bit] = readLE16(&valuesStart[byteOffset]);
      byteOffset += 2U;
      channelCount++;
    }
  }

  return channelCount;
}


/**
* @brief   Compute the total frame length a Control Data packet with the given mask should have.
* @param   mask  The frame's channel mask field.
* @return  CONTROL_HEADER_LENGTH + 2 bytes per active channel + CRC_LENGTH.
*/
uint32_t
Srxl2::controlFrameExpectedLength(const uint32_t mask)
{
  return CONTROL_HEADER_LENGTH + (countActiveChannels(mask) * 2U) + CRC_LENGTH;
}


/**
* @brief   Build a complete Handshake packet into buf.
* @details Always advertises OUR_DEVICE_ID, OUR_PRIORITY, 115200-only baud
*          support, and INFO_NO_TELEMETRY (never asks to be polled for telemetry).
* @param   buf        Output buffer; must be at least HANDSHAKE_FRAME_LENGTH bytes.
* @param   destDevId  Device ID to address the handshake to.
* @param   uid       32-bit value to advertise as this device's UID.
* @return  Number of bytes written (always HANDSHAKE_FRAME_LENGTH).
*/
uint32_t
Srxl2::buildHandshakeFrame(uint8_t* const buf, const uint8_t destDevId, const uint32_t uid)
{
  buf[SYNC_OFFSET]   = SRXL_SYNC_BYTE;
  buf[TYPE_OFFSET]   = PACKET_TYPE_HANDSHAKE;
  buf[LENGTH_OFFSET] = static_cast<uint8_t>(HANDSHAKE_FRAME_LENGTH);

  buf[HANDSHAKE_SRC_OFFSET]      = OUR_DEVICE_ID;
  buf[HANDSHAKE_DEST_OFFSET]     = destDevId;
  buf[HANDSHAKE_PRIORITY_OFFSET] = OUR_PRIORITY;
  buf[HANDSHAKE_BAUD_OFFSET]     = BAUD_SUPPORTED_115200_ONLY;
  buf[HANDSHAKE_INFO_OFFSET]     = INFO_NO_TELEMETRY;
  writeLE32(&buf[HANDSHAKE_UID_OFFSET], uid);

  const uint16_t crc = calculateCrc(buf, HANDSHAKE_FRAME_LENGTH - CRC_LENGTH);
  buf[HANDSHAKE_FRAME_LENGTH - 2U] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
  buf[HANDSHAKE_FRAME_LENGTH - 1U] = static_cast<uint8_t>(crc & 0xFFU);

  return HANDSHAKE_FRAME_LENGTH;
}


// =============================================================================
// Transport: UART ownership, byte-stream state machine, RxBase overrides.
// =============================================================================

/**
* @brief    SRXL2 constructor. See Srxl2.hpp for the half-duplex wiring note.
*/
Srxl2::Srxl2()
{
  m_uart = InternalConfig::resolveUart(Config::RECEIVER_SERIAL_PORT);
  m_uart->begin(SRXL2_BAUD, SERIAL_8N1,
                InternalConfig::resolveRxPin(Config::RECEIVER_SERIAL_PORT),
                InternalConfig::resolveTxPin(Config::RECEIVER_SERIAL_PORT),
                false);
  m_uart->flush();
}


/**
* @brief    Decodes the SRXL2 serial stream into usable channel and flag data.
* @details  Implements a byte-stream parser keyed on SRXL2's 3-byte header
*           (sync, packet type, length): hunt for sync, buffer the type byte,
*           range-check the length byte, then accumulate until the frame is
*           complete, validate its CRC, and dispatch by packet type. While
*           the bus is quiet it also announces this device with a handshake
*           every UNPROMPTED_HANDSHAKE_PERIOD_MS.
* @return  true when a new, valid Channel Data / Failsafe Channel Data frame
*           has been decoded and m_rxData updated; false on all other calls.
*/
bool
Srxl2::getDemands()
{
  uint32_t currentByte;
  uint32_t rxCount = 0U;

  while (m_uart->available() && (rxCount < MAX_BYTES_PER_LOOP))
  {
    currentByte = m_uart->read();
    rxCount++;

    if (0U == m_bufferIndex)
    {
      // Hunt for a valid sync byte.
      if (currentByte == static_cast<uint32_t>(SRXL_SYNC_BYTE))
      {
        m_buffer[m_bufferIndex] = static_cast<uint8_t>(currentByte);
        m_bufferIndex++;
      }
    }
    else if (1U == m_bufferIndex)
    {
      // Packet-type byte: any value accepted here; unrecognised types are
      // discarded once the frame is complete, in processFrame().
      m_buffer[m_bufferIndex] = static_cast<uint8_t>(currentByte);
      m_bufferIndex++;
    }
    else if (2U == m_bufferIndex)
    {
      // Frame-length byte, range-checked before committing to buffering
      // that many bytes.
      if ((currentByte < MIN_FRAME_LENGTH) || (currentByte > MAX_FRAME_SIZE))
      {
        m_bufferIndex = 0U;   // Not plausible - discard and restart hunt.
      }
      else
      {
        m_buffer[m_bufferIndex] = static_cast<uint8_t>(currentByte);
        m_bufferIndex++;
      }
    }
    else
    {
      // Accumulate until the frame is complete.
      m_buffer[m_bufferIndex] = static_cast<uint8_t>(currentByte);
      m_bufferIndex++;

      if (m_bufferIndex >= static_cast<uint32_t>(m_buffer[LENGTH_OFFSET]))
      {
        const uint32_t frameLength = static_cast<uint32_t>(m_buffer[LENGTH_OFFSET]);
        m_bufferIndex = 0U;   // Frame complete either way - resync for the next one.

        if (isFrameCrcValid(m_buffer, frameLength))
        {
          if (processFrame(m_buffer, frameLength))
          {
            return true;
          }
        }
      }
    }
  }

  // Listen-only receivers stay silent until a bus device speaks first, so
  // announce ourselves to the (as yet unknown) master while the bus is quiet.
  const uint32_t nowMs = millis();
  if (((nowMs - m_lastBusActivityMs) >= UNPROMPTED_HANDSHAKE_PERIOD_MS)
      && ((nowMs - m_lastHandshakeSentMs) >= UNPROMPTED_HANDSHAKE_PERIOD_MS))
  {
    sendHandshake(UNKNOWN_MASTER_ID);
    m_lastHandshakeSentMs = nowMs;
  }

  if (CTimer::State::EXPIRED == m_lossOfCommsTimer.getState())
  {
    m_rxData.lostComms = true;
  }

  return false;
}


/**
* @brief    Dispatch a complete, CRC-valid frame by packet type.
* @param    buffer  The frame buffer, starting at the sync byte.
* @param    length  Total frame length.
* @return   true if this frame updated m_rxData with new channel data.
*/
bool
Srxl2::processFrame(const uint8_t* const buffer, const uint32_t length)
{
  bool newData = false;
  const uint8_t packetType = buffer[TYPE_OFFSET];

  // Our own handshake echoes back onto RX on the shared wire; it is not bus activity.
  const bool ownEcho = (PACKET_TYPE_HANDSHAKE == packetType)
                    && (OUR_DEVICE_ID == buffer[HANDSHAKE_SRC_OFFSET]);
  if (!ownEcho)
  {
    m_lastBusActivityMs = millis();
  }

  if (PACKET_TYPE_HANDSHAKE == packetType)
  {
    handleHandshake(buffer, length);
  }
  else if (PACKET_TYPE_CONTROL == packetType)
  {
    newData = handleControlData(buffer, length);
  }
  else
  {
    // Bind, RSSI request, Telemetry request, Parameter and any other packet
    // types are ignored.
  }

  return newData;
}


/**
* @brief    Decode a Channel Data / Failsafe Channel Data Control Data frame.
* @param    buffer  The frame buffer, starting at the sync byte.
* @param    length  Total frame length.
* @return   true if m_rxData was updated with new channel data.
*/
bool
Srxl2::handleControlData(const uint8_t* const buffer, const uint32_t length)
{
  bool newData = false;

  if (length >= (CONTROL_HEADER_LENGTH + CRC_LENGTH))
  {
    const uint8_t cmd = buffer[CONTROL_CMD_OFFSET];

    if ((CTRL_CMD_CHANNEL == cmd) || (CTRL_CMD_CHANNEL_FS == cmd))
    {
      const uint32_t mask = readLE32(&buffer[CONTROL_MASK_OFFSET]);

      // A frame with an empty channel mask carries no channel data (the
      // receiver sends these while it has no link to the Tx, e.g. after
      // restarting with the Tx off). It must not clear failsafe or count
      // as a sign of life.
      if ((0U != mask) && (length == controlFrameExpectedLength(mask)))
      {
        uint32_t rawValues[MAX_SRXL_CHANNELS];
        unpackChannelValues(&buffer[CONTROL_VALUES_OFFSET], mask, rawValues);

        for (uint32_t bit = 0U; bit < MAX_SRXL_CHANNELS; bit++)
        {
          if (((mask & (1UL << bit)) != 0U) && (bit < MAX_RX_CHANNELS))
          {
            const uint32_t constrained = constrain(rawValues[bit], MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE);
            m_rxData.ch[bit] = map32(constrained, MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE,
                                      MIN_NORMALISED, MAX_NORMALISED);
          }
        }

        m_rxData.failsafe = (CTRL_CMD_CHANNEL_FS == cmd);
        m_rssi = static_cast<int8_t>(buffer[CONTROL_RSSI_OFFSET]);

        // Reset communications timer
        m_lossOfCommsTimer.set(COMMS_TIME_OUT_PERIOD);
        m_rxData.lostComms = false;

        if constexpr (InternalConfig::DEBUG_RX)
        {
          printData();
        }

        newData = true;
      }
      // else: frame length doesn't match the channel mask despite a valid
      // CRC - shouldn't happen; discard silently.
    }
  }

  return newData;
}


/**
* @brief    Handle a Handshake frame witnessed on the bus.
* @details  Only the device a Handshake is addressed to may transmit on the
*           shared bus, so this replies only when destDevID is this device's
*           ID, and never to a broadcast (0xFF). Unprompted handshakes are
*           sent from getDemands() while the bus is quiet. This device is
*           never the bus master, so there is no bus-arbitration logic here.
* @param    buffer  The frame buffer, starting at the sync byte.
* @param    length  Total frame length.
*/
void
Srxl2::handleHandshake(const uint8_t* const buffer, const uint32_t length)
{
  if (HANDSHAKE_FRAME_LENGTH == length)
  {
    // Only the addressed device may transmit on the shared bus.
    // Broadcast handshakes (dest 0xFF) are never answered.
    if (OUR_DEVICE_ID == buffer[HANDSHAKE_DEST_OFFSET])
    {
      sendHandshake(buffer[HANDSHAKE_SRC_OFFSET]);
    }
  }
}


/**
* @brief    Build and transmit our own Handshake packet.
* @param    destDevId  Device ID to address the handshake to.
*/
void
Srxl2::sendHandshake(const uint8_t destDevId)
{
  uint8_t buf[HANDSHAKE_FRAME_LENGTH];
  const uint32_t length = buildHandshakeFrame(buf, destDevId, deviceUid());

  m_uart->write(buf, length);
}


/**
* @brief    A statistically-unique 32-bit value to advertise as this device's UID.
* @return   The low 32 bits of the ESP32's factory-programmed MAC address.
* @note     Not required to be globally unique per spec, only statistically
*           unique across devices on this bus - the efuse MAC is a convenient,
*           always-available source.
*/
uint32_t
Srxl2::deviceUid()
{
  return static_cast<uint32_t>(ESP.getEfuseMac());
}


/**
* @brief    Returns the most recent receiver data.
* @return   RxPacket containing failsafe status, communication status, and normalised channel data.
*/
const RxBase::RxPacket
Srxl2::getData()
{
  return m_rxData;
}


/**
* @brief    Check if communications have been lost.
* @return   true when SRXL2 comms has been lost, false otherwise.
*/
const bool
Srxl2::hasLostCommunications()
{
  return m_rxData.lostComms;
}


/**
* @brief    Prints SRXL2 data to console for debugging.
*/
void
Srxl2::printData(void)
{
  if constexpr (InternalConfig::DEBUG_RX)
  {
    static uint64_t lastPrintTime = 0U;   // Static is safe: only one Srxl2 instance exists.
    const uint64_t  now   = esp_timer_get_time();
    const uint64_t  delta = now - lastPrintTime;
    lastPrintTime = now;

    const uint32_t hz = static_cast<uint32_t>((delta > 0U) ? (1000000U / delta) : 0U);
    Serial.print("Hz=");
    Serial.print(hz);
    Serial.print("\t");

    for (uint32_t i = 0U; i < MAX_RX_CHANNELS; i++)
    {
      Serial.print(m_rxData.ch[i]);
      Serial.print("\t");
    }

    Serial.print(m_rxData.failsafe);
    Serial.print("\t");
    Serial.println(m_rxData.lostComms);
  }
}
