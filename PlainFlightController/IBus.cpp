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
* @file   IBus.cpp
* @brief  This class handles communications with an IBus RC receiver.
*/

#include "IBus.hpp"
#include "InternalConfig.hpp"
 

/**
* @brief    IBus constructor.
*/
IBus::IBus()
{
  m_uart = InternalConfig::resolveUart(Config::RECEIVER_SERIAL_PORT);
  m_uart->begin(IBUS_BAUD, SERIAL_8N1,
                InternalConfig::resolveRxPin(Config::RECEIVER_SERIAL_PORT),
                InternalConfig::resolveTxPin(Config::RECEIVER_SERIAL_PORT),
                false);
  m_uart->flush();
}


/**
* @brief    Decodes the IBus serial stream into usable channel and flag data.
* @return   Returns true when new data is available.
*/
bool
IBus::getDemands()
{
  uint32_t currentByte;
  uint32_t rxCount = 0U;
  uint32_t rawChannels[NUM_IBUS_CH + CHECKSUM_LEN];
 
  //We loop for a maximum of MAX_BYTES_PER_LOOP bytes then get out of here to avoid blocking main loop.
  while (m_uart->available() && (rxCount < MAX_BYTES_PER_LOOP))
  {
    currentByte = m_uart->read();
    rxCount++;
 
    if (0U == m_count)
    {
      if ((COMMAND == currentByte) && (HEADER == m_prevByte))
      {
        m_buff[m_count] = HEADER;
        m_count++;
        m_buff[m_count] = COMMAND;
        m_count++;
      }
    }
    else if (m_count < FRAME_LEN)
    {
      m_buff[m_count] = currentByte;
      m_count++;
 
      if (FRAME_LEN == m_count)
      {
        m_count = 0U;
        const uint16_t rxChecksum =  static_cast<uint32_t>(m_buff[30] | (m_buff[31] << 8U));
        uint16_t calcChecksum = CHECKSUM_START;
 
        for (uint16_t i=0U; i<CHECKSUM_BYTE_LEN; i++)
        {
          calcChecksum -= m_buff[i];
        }

        if (rxChecksum == calcChecksum)
        {
          //Checksum ok so decode the payload
          //Reset communications timer
          lossOfCommsTimer.set(COMMS_TIME_OUT_PERIOD);
          m_rxData.lostComms = false;

          //There is no failsafe bit so set false
          m_rxData.failsafe = false;

          //Decode data
          rawChannels[0] =  static_cast<uint32_t>((m_buff[2]  | (m_buff[3] << 8U))  & 0x07FFU);
          rawChannels[1] =  static_cast<uint32_t>((m_buff[4]  | (m_buff[5] << 8U))  & 0x07FFU);
          rawChannels[2] =  static_cast<uint32_t>((m_buff[6]  | (m_buff[7] << 8U))  & 0x07FFU);
          rawChannels[3] =  static_cast<uint32_t>((m_buff[8]  | (m_buff[9] << 8U))  & 0x07FFU);
          rawChannels[4] =  static_cast<uint32_t>((m_buff[10] | (m_buff[11] << 8U)) & 0x07FFU);
          rawChannels[5] =  static_cast<uint32_t>((m_buff[12] | (m_buff[13] << 8U)) & 0x07FFU);
          rawChannels[6] =  static_cast<uint32_t>((m_buff[14] | (m_buff[15] << 8U)) & 0x07FFU);
          rawChannels[7] =  static_cast<uint32_t>((m_buff[16] | (m_buff[17] << 8U)) & 0x07FFU);
          rawChannels[8] =  static_cast<uint32_t>((m_buff[18] | (m_buff[19] << 8U)) & 0x07FFU);
          rawChannels[9] =  static_cast<uint32_t>((m_buff[20] | (m_buff[21] << 8U)) & 0x07FFU);
          rawChannels[10] = static_cast<uint32_t>((m_buff[22] | (m_buff[23] << 8U)) & 0x07FFU);
          rawChannels[11] = static_cast<uint32_t>((m_buff[24] | (m_buff[25] << 8U)) & 0x07FFU);
          rawChannels[12] = static_cast<uint32_t>((m_buff[26] | (m_buff[27] << 8U)) & 0x07FFU);
          rawChannels[13] = static_cast<uint32_t>((m_buff[28] | (m_buff[29] << 8U)) & 0x07FFU);

          for (uint32_t i = 0U; i < NUM_IBUS_CH; i++)
          {
            const int32_t channel = constrain(rawChannels[i], MIN_IBUS_US, MAX_IBUS_US);
            m_rxData.ch[i] = map32(channel, MIN_IBUS_US, MAX_IBUS_US, MIN_NORMALISED, MAX_NORMALISED);
          }

          if constexpr(InternalConfig::DEBUG_RX)
          {
            printData();
          }

          return true;
        }
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
    m_rxData.failsafe = true;   //Force failsafe
  }
 
  return false;
}
 
/**
* @brief    Returns the most recent receiver data.
* @return   RxPacket containing failsafe, communication status, and normalised channel data.
*/
const RxBase::RxPacket
IBus::getData()
{
  return m_rxData;
}
 
/**
* @brief    Check if communications have been lost.
* @return   true when IBus comms has been lost, false otherwise.
*/
const bool
IBus::hasLostCommunications()
{
  return m_rxData.lostComms;
}
 
/**
* @brief    Prints IBus data to console for debugging.
*/
void
IBus::printData(void)
{
  if constexpr (InternalConfig::DEBUG_RX)
  {
    static uint64_t lastPrintTime = 0U;     //Use of static ok here as there will only ever be one IBus class.
    const uint64_t now = esp_timer_get_time();
    const uint64_t delta = now - lastPrintTime;
    lastPrintTime = now;
 
    // Display update time delta in microseconds
    const uint32_t hz = static_cast<uint32_t>((delta > 0U) ? (1000000U / delta) : 0U);
    Serial.print("Hz=");
    Serial.print(hz, 1);
    Serial.print("\t");
 
    // Display the received normalised data
    for (uint32_t i = 0U; i < NUM_IBUS_CH; i++)
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
}
