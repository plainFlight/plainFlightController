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
* @file   SBus.cpp
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

#include "SBus.hpp"
#include "Utilities.hpp"


/**
* @brief    SBus constructor.
* @param    *uart - Pointer to hardware serial port.
* @param    rxPin - RX pin number.
* @param    txPin - TX pin number.
*/
SBus::SBus(HardwareSerial *uart, uint8_t rxPin, uint8_t txPin)
{
  m_uart = uart;
  m_uart->begin(SBUS_BAUD, SERIAL_8E2, rxPin, txPin, true);
  m_uart->flush();
}


/**
* @brief    Decodes the SBus serial stream into usable channel and flag data.
* @return   Returns true when new data is available.
*/
bool
SBus::getDemands()
{
  uint32_t currentByte;
  uint32_t rxCount = 0U;
  uint32_t rawChannels[NUM_SBUS_CH];

  //We loop for a maximum of MAX_BYTES_PER_LOOP bytes then get out of here to avoid blocking main loop.
  while (m_uart->available() && (rxCount < MAX_BYTES_PER_LOOP))
  {
    currentByte = m_uart->read();
    rxCount++;

    if (0U == m_count)
    {
      if ((currentByte == HEADER) && ((m_prevByte == FOOTER) || ((m_prevByte & 0x0FU) == FOOTER2)))
      {
        m_buff[m_count] = currentByte;
        m_count++;
      }
    }
    else if (m_count < (PAYLOAD_LEN + HEADER_LEN))
    {
      m_buff[m_count] = currentByte;
      m_count++;
    }
    else if (m_count < (PAYLOAD_LEN + HEADER_LEN + FOOTER_LEN))
    {
      m_count = 0U;
      m_prevByte = currentByte;
      if ((currentByte == FOOTER) || ((currentByte & 0x0FU) == FOOTER2))
      {
        // Decode SBus data into raw channel values
        rawChannels[0] = (uint32_t)(m_buff[1] | ((m_buff[2] << 8U) & 0x07FFU));
        rawChannels[1] = (uint32_t)((m_buff[2] >> 3U) | ((m_buff[3] << 5U) & 0x07FFU));
        rawChannels[2] = (uint32_t)((m_buff[3] >> 6U) | (m_buff[4] << 2U) | ((m_buff[5] << 10U) & 0x07FFU));
        rawChannels[3] = (uint32_t)((m_buff[5] >> 1U) | ((m_buff[6] << 7U) & 0x07FFU));
        rawChannels[4] = (uint32_t)((m_buff[6] >> 4U) | ((m_buff[7] << 4U) & 0x07FFU));
        rawChannels[5] = (uint32_t)((m_buff[7] >> 7U) | (m_buff[8] << 1U) | ((m_buff[9] << 9U) & 0x07FFU));
        rawChannels[6] = (uint32_t)((m_buff[9] >> 2U) | ((m_buff[10] << 6U) & 0x07FFU));
        rawChannels[7] = (uint32_t)((m_buff[10] >> 5U) | ((m_buff[11] << 3U) & 0x07FFU));

        if constexpr(Config::USE_PROP_HANG_MODE)
        {
          rawChannels[8] = (uint32_t)(m_buff[12] | ((m_buff[13] << 8U) & 0x07FFU));
        }
        else
        {
          if constexpr(USE_ALL_18_CHANNELS)  //Speeds up main loop by not processing all 18 channels
          {
            static_assert(!USE_ALL_18_CHANNELS || (MAX_RX_CHANNELS >= 18), "MAX_RC_CHANNELS must be at least 18 when USE_ALL_18_CHANNELS is true");
            rawChannels[8] = (uint32_t)(m_buff[12] | ((m_buff[13] << 8U) & 0x07FFU));
            rawChannels[9] = (uint32_t)((m_buff[13] >> 3U) | ((m_buff[14] << 5U) & 0x07FFU));
            rawChannels[10] = (uint32_t)((m_buff[14] >> 6U) | (m_buff[15] << 2U) | ((m_buff[16] << 10U) & 0x07FFU));
            rawChannels[11] = (uint32_t)((m_buff[16] >> 1U) | ((m_buff[17] << 7U) & 0x07FFU));
            rawChannels[12] = (uint32_t)((m_buff[17] >> 4U) | ((m_buff[18] << 4U) & 0x07FFU));
            rawChannels[13] = (uint32_t)((m_buff[18] >> 7U) | (m_buff[19] << 1U) | ((m_buff[20] << 9U) & 0x07FFU));
            rawChannels[14] = (uint32_t)((m_buff[20] >> 2U) | ((m_buff[21] << 6U) & 0x07FFU));
            rawChannels[15] = (uint32_t)((m_buff[21] >> 5U) | ((m_buff[22] << 3U) & 0x07FFU));
            rawChannels[17] = (m_buff[23] & CH17_MASK) ? MAX_SBUS_US : MIN_SBUS_US;  // No special boolean channels
            rawChannels[18] = (m_buff[23] & CH18_MASK) ? MAX_SBUS_US : MIN_SBUS_US;  // No special boolean channels
          }  
        } 

        // Normalise channels immediately after decoding
        uint32_t numChannels = 8U;
        if constexpr (Config::USE_PROP_HANG_MODE)
        {
          numChannels = 9U;
        }
        else if constexpr (USE_ALL_18_CHANNELS)
        {
          numChannels = 18U;
        }

        for (uint32_t i = 0U; i < numChannels; i++)
        {
          m_rxData.ch[i] = map32(rawChannels[i], MIN_SBUS_US, MAX_SBUS_US, MIN_NORMALISED, MAX_NORMALISED);
        }

        // Decode lost frame bit
        m_lostFrame = (m_buff[23] & LOST_FRAME_MASK) ? true : false;

        // Decode failsafe bit
        m_rxData.failsafe = (m_buff[23] & FAILSAFE_MASK) ? true : false;

        // Reset communications timer
        lossOfCommsTimer.set(COMMS_TIME_OUT_PERIOD);
        m_rxData.lostComms = false;

        if constexpr(Config::DEBUG_RX)
        {
          printData();
        }

        return true;
      }
    } 
    else
    {
      m_count = 0U;
    }

    m_prevByte = currentByte;
  }

  if (CTimer::State::EXPIRED == lossOfCommsTimer.getState())
  {
    m_rxData.lostComms = true;
  }

  return false;
}


/**
* @brief    Returns the most recent receiver data.
* @return   RxPacket containing failsafe, communication status, and normalised channel data.
*/
const RxBase::RxPacket
SBus::getData() const
{
  return m_rxData;
}


/**
* @brief    Check if communications have been lost.
* @return   true when SBus comms has been lost, false otherwise.
*/
const bool
SBus::hasLostCommunications() const
{
  return m_rxData.lostComms;
}


/**
* @brief    Prints SBus data to console for debugging.
*/
void
SBus::printData(void)
{
  static uint64_t lastPrintTime = 0U;     //Use of static ok here as there will only ever be one SBus class.
  const uint64_t now = esp_timer_get_time();
  const uint64_t delta = now - lastPrintTime;
  lastPrintTime = now;

  // Display update time delta in microseconds
  const uint32_t hz = static_cast<uint32_t>((delta > 0U) ? (1000000U / delta) : 0U);
  Serial.print("Hz=");
  Serial.print(hz, 1);
  Serial.print("\t");

  uint32_t channels;

  if constexpr(USE_ALL_18_CHANNELS)
  {
    channels = 16U;
  }
  else
  {
    if constexpr(Config::USE_PROP_HANG_MODE)
    {
      channels = 9U;
    }
    else
    {
      channels = 8U;
    }
  }

  // Display the received normalised data
  for (uint32_t i = 0U; i < channels; i++)
  {
    Serial.print(m_rxData.ch[i]);
    Serial.print("\t");
  }
  // Display lost frames and failsafe data
  Serial.print(m_lostFrame);
  Serial.print("\t");
  Serial.print(m_rxData.failsafe);
  Serial.print("\t");
  Serial.println(m_rxData.lostComms);
}