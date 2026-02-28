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
* @brief  This class handles communications with a CRSF/ELRS RC receiver.
*/

#include "Crsf.hpp"
#include "Utilities.hpp"


/**
* @brief    CRSF constructor.
* @param    uart - Pointer to hardware serial port.
* @param    rxPin - RX pin number.
* @param    txPin - TX pin number.
*/
Crsf::Crsf(HardwareSerial *uart, uint8_t rxPin, uint8_t txPin)
{
  m_uart = uart;
  m_uart->begin(CRSF_BAUD, SERIAL_8N1, rxPin, txPin, false);
  m_uart->flush();
}


/**
* @brief    Decodes the CRSF serial stream into usable channel and flag data.
* @return   Returns true when new data is available.
*/
bool
Crsf::getDemands()
{
  uint32_t currentByte;
  uint32_t rxCount = 0U;

  // We loop for a maximum of MAX_BYTES_PER_LOOP bytes then get out of here to avoid blocking main loop.
  while (m_uart->available() && (rxCount < MAX_BYTES_PER_LOOP))
  {
    currentByte = m_uart->read();
    rxCount++;

    // State 0: Looking for sync byte (device address)
    if (0U == m_bufferIndex)
    {
      if ((currentByte == CRSF_ADDRESS_FLIGHT_CONTROLLER) ||
          (currentByte == CRSF_ADDRESS_ALTERNATIVE))
      {
        m_buffer[m_bufferIndex] = currentByte;
        m_bufferIndex++;
      }
    }
    // State 1: Get frame length
    else if (1U == m_bufferIndex)
    {
      // Frame length includes type + payload + crc, should be reasonable
      if ((currentByte >= 2U) && (currentByte <= (CRSF_MAX_FRAME_SIZE - 2U)))
      {
        m_buffer[m_bufferIndex] = currentByte;
        m_bufferIndex++;
      }
      else
      {
        // Invalid length, reset state machine
        m_bufferIndex = 0U;
      }
    }
    // State 2+: Collect frame data
    else
    {
      m_buffer[m_bufferIndex] = currentByte;
      m_bufferIndex++;
      
      // Check if we have complete frame (ADDR + LEN + frame_length bytes)
      uint32_t frameLength = m_buffer[CRSF_LENGTH_OFFSET];
      if (m_bufferIndex >= (frameLength + 2U))
      {
        // Validate CRC
        uint32_t crcIndex = frameLength + 1U;
        uint32_t calculatedCrc = calculateCrc(&m_buffer[CRSF_FRAMETYPE_OFFSET], frameLength - 1U);

        if (calculatedCrc == m_buffer[crcIndex])
        {
          // Valid frame - process based on type
          uint32_t frameType = m_buffer[CRSF_FRAMETYPE_OFFSET];
          const uint8_t *payload = &m_buffer[CRSF_PAYLOAD_OFFSET];
          uint8_t payloadLength = m_buffer[CRSF_LENGTH_OFFSET];
          
          if (CRSF_FRAMETYPE_RC_CHANNELS == frameType)
          {
            parseRcChannels(payload, payloadLength);
            
            // Reset communications timer
            lossOfCommsTimer.set(COMMS_TIME_OUT_PERIOD);
            m_rxData.lostComms = false;

            if constexpr(Config::DEBUG_RX)
            {
              printData();
            }

            // Reset buffer for next frame
            m_bufferIndex = 0U;
            
            return true;
          }
          else if (CRSF_FRAMETYPE_LINK_STATS == frameType)
          {
            parseLinkStatistics(payload, payloadLength);
          }
          else
          {
            // Unknown frame type - ignore
          }
        }
        
        // Reset buffer for next frame
        m_bufferIndex = 0U;
      }
      
      // Prevent buffer overflow
      if (m_bufferIndex >= CRSF_MAX_FRAME_SIZE)
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
* @brief    Parses and normalises RC channels from CRSF payload.
* @param    payload - Pointer to RC channels payload (22 bytes).
* @param    payloadLength - Length of payload.
*/
void
Crsf::parseRcChannels(const uint8_t *payload, uint8_t payloadLength)
{
  uint32_t rawChannels[NUM_CRSF_CH];

  // Verify we have enough data for 16 channels (22 bytes)
  if (payloadLength < CRSF_RC_FRAME_SIZE)
  {
    return;
  }

  // Unpack 11-bit channels from packed byte array
  // CRSF channel packing is little-endian, LSB first
  rawChannels[0]  = ((payload[0]       | (payload[1]  << 8U))                           & 0x07FFU);
  rawChannels[1]  = (((payload[1] >> 3U) | (payload[2]  << 5U))                         & 0x07FFU);
  rawChannels[2]  = (((payload[2] >> 6U) | (payload[3]  << 2U) | (payload[4] << 10U))  & 0x07FFU);
  rawChannels[3]  = (((payload[4] >> 1U) | (payload[5]  << 7U))                         & 0x07FFU);
  rawChannels[4]  = (((payload[5] >> 4U) | (payload[6]  << 4U))                         & 0x07FFU);
  rawChannels[5]  = (((payload[6] >> 7U) | (payload[7]  << 1U) | (payload[8] << 9U))   & 0x07FFU);
  rawChannels[6]  = (((payload[8] >> 2U) | (payload[9]  << 6U))                         & 0x07FFU);
  rawChannels[7]  = (((payload[9] >> 5U) | (payload[10] << 3U))                         & 0x07FFU);
  rawChannels[8]  = ((payload[11]      | (payload[12] << 8U))                           & 0x07FFU);
  rawChannels[9]  = (((payload[12] >> 3U) | (payload[13] << 5U))                        & 0x07FFU);
  rawChannels[10] = (((payload[13] >> 6U) | (payload[14] << 2U) | (payload[15] << 10U)) & 0x07FFU);
  rawChannels[11] = (((payload[15] >> 1U) | (payload[16] << 7U))                        & 0x07FFU);
  rawChannels[12] = (((payload[16] >> 4U) | (payload[17] << 4U))                        & 0x07FFU);
  rawChannels[13] = (((payload[17] >> 7U) | (payload[18] << 1U) | (payload[19] << 9U))  & 0x07FFU);
  rawChannels[14] = (((payload[19] >> 2U) | (payload[20] << 6U))                        & 0x07FFU);
  rawChannels[15] = (((payload[20] >> 5U) | (payload[21] << 3U))                        & 0x07FFU);

  // Normalise all channels to -1024/+1024 range
  for (uint32_t i = 0U; i < NUM_CRSF_CH; i++)
  {
    m_rxData.ch[i] = map32(rawChannels[i], MIN_CRSF_US, MAX_CRSF_US, MIN_NORMALISED, MAX_NORMALISED);
  }

  // Multi-level failsafe detection
  m_rxData.failsafe = m_rxData.lostComms || (m_crsfLinkStats.linkQuality < FAILSAFE_LQ_THRESHOLD);
}


/**
* @brief    Parses link statistics from CRSF payload.
* @param    payload - Pointer to link statistics payload.
* @param    payloadLength - Length of payload.
*/
void
Crsf::parseLinkStatistics(const uint8_t *payload, uint8_t payloadLength)
{
  // Verify we have enough data for link statistics (10 bytes minimum)
  if (payloadLength < CRSF_LINK_FRAME_SIZE)
  {
    return;
  }

  // Store uplink (receiver to transmitter) statistics
  m_crsfLinkStats.rssiDbm = static_cast<int8_t>(payload[0]);
  m_crsfLinkStats.linkQuality = static_cast<uint8_t>(payload[2]);
  m_crsfLinkStats.snrDb = static_cast<int8_t>(payload[3]);
  m_crsfLinkStats.txPower = static_cast<uint8_t>(payload[6]);
}


/**
* @brief    Calculate CRC8 for a single byte (DVB-S2 polynomial).
* @param    crc - Current CRC value.
* @param    a - Byte to process.
* @return   Updated CRC8 value.
*/
uint8_t
Crsf::crc8_dvb_s2(uint8_t crc, uint8_t a) const
{
  crc ^= a;
  for (uint32_t i = 0U; i < 8U; i++)
  {
    if ((crc & 0x80U) != 0U)
    {
      crc = (crc << 1U) ^ 0xD5U;
    }
    else
    {
      crc = crc << 1U;
    }
  }
  return crc;
}


/**
* @brief    Calculate CRC8 for a data buffer.
* @param    data - Pointer to data buffer.
* @param    length - Length of data.
* @return   CRC8 value.
*/
uint8_t
Crsf::calculateCrc(const uint8_t *data, uint8_t length) const
{
  uint32_t crc = 0U;
  for (uint32_t i = 0U; i < length; i++)
  {
    crc = crc8_dvb_s2(crc, data[i]);
  }
  return crc;
}


/**
* @brief    Returns the most recent receiver data.
* @return   RxPacket containing failsafe, communication status, and normalised channel data.
*/
const RxBase::RxPacket
Crsf::getData() const
{
  return m_rxData;
}


/**
* @brief    Check if communications have been lost.
* @return   true when CRSF comms has been lost, false otherwise.
*/
const bool
Crsf::hasLostCommunications() const
{
  return m_rxData.lostComms;
}


/**
* @brief    Returns the CRSF-specific link statistics.
* @return   CrsfLinkStats structure containing link quality data.
*/
Crsf::CrsfLinkStats
Crsf::getLinkStats() const
{
  return m_crsfLinkStats;
}


/**
* @brief    Prints CRSF data to console for debugging.
*/
void
Crsf::printData(void)
{
  static uint32_t lastPrintTime = 0U;
  uint32_t now = micros();
  uint32_t delta = now - lastPrintTime;
  lastPrintTime = now;

  // Display update time delta in microseconds
  uint32_t hz = (delta > 0U) ? (1000000U / delta) : 0U;
  Serial.print("Hz=");
  Serial.print(hz, 1);
  Serial.print("\t");
  
  // Display the received normalised data
  for (uint32_t i = 0U; i < NUM_CRSF_CH; i++)
  {
    Serial.print(m_rxData.ch[i]);
    Serial.print("\t");
  }
  // Display failsafe and communications status
  Serial.print(m_rxData.failsafe);
  Serial.print("\t");
  Serial.println(m_rxData.lostComms);
}
