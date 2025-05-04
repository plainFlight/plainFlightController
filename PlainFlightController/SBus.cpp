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
* @file   Sbus.cpp
* @brief  This class handles communications with an Sbus Rx.
*/

/*
* While very different, this module has been inspired by and gives credit to:
*
* Brian R Taylor
* brian.taylor@bolderflight.com
* Copyright (c) 2022 Bolder Flight Systems Inc
* https://github.com/bolderflight/sbus
*/

#include "Sbus.hpp"


/**
* @brief    SBus constructor  
* @param    *uart, rxPin, txPin
*/  
SBus::SBus(HardwareSerial *uart, uint8_t rxPin, uint8_t txPin)
{
  m_uart = uart;
  m_uart->begin(SBUS_BAUD, SERIAL_8E2, rxPin, txPin, true);
  m_uart->flush();  
}


/**
* @brief    Decodes the SBus serial stream into usable channel and flag data.
*/  
bool    
SBus::getDemands()
{
  uint32_t currentByte;
  uint32_t rxCount = 0;

  //We loop for a maximum of 6 bytes then get out of here to avoid blocking main loop.
  while (m_uart->available() && (rxCount < 6)) 
  {
    currentByte = m_uart->read();
    rxCount++;

    if (0U == m_count) 
    {
      if ((currentByte == HEADER) && ((m_prevByte == FOOTER) || ((m_prevByte & 0x0F) == FOOTER2))) 
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
      if ((currentByte == FOOTER) || ((currentByte & 0x0F) == FOOTER2)) 
      {
        //Decode sbus data
        m_sbus.ch[0]  = (uint32_t)(m_buff[1] | ((m_buff[2] << 8U) & 0x07FF));
        m_sbus.ch[1]  = (uint32_t)((m_buff[2] >> 3U) | ((m_buff[3] << 5U) & 0x07FF));
        m_sbus.ch[2]  = (uint32_t)((m_buff[3] >> 6U) | (m_buff[4] << 2U) | ((m_buff[5] << 10U) & 0x07FF));
        m_sbus.ch[3]  = (uint32_t)((m_buff[5] >> 1U) | ((m_buff[6] << 7U) & 0x07FF));
        m_sbus.ch[4]  = (uint32_t)((m_buff[6] >> 4U) | ((m_buff[7] << 4U) & 0x07FF));
        m_sbus.ch[5]  = (uint32_t)((m_buff[7] >> 7U) | (m_buff[8] << 1U) | ((m_buff[9] << 9U) & 0x07FF));
        m_sbus.ch[6]  = (uint32_t)((m_buff[9] >> 2U) | ((m_buff[10] << 6U) & 0x07FF));
        m_sbus.ch[7]  = (uint32_t)((m_buff[10] >> 5U) | ((m_buff[11] << 3U) & 0x07FF));
        if constexpr(Config::USE_PROP_HANG_MODE)
        {
          m_sbus.ch[8]  = (uint32_t)(m_buff[12] | ((m_buff[13] << 8U) & 0x07FF));
        }
        else
        {
          if constexpr(USE_ALL_18_CHANNELS)  //Speeds up main loop by not processing all 18 channels
          {
            m_sbus.ch[8]  = (uint32_t)(m_buff[12] | ((m_buff[13] << 8U) & 0x07FF));
            m_sbus.ch[9]  = (uint32_t)((m_buff[13] >> 3U) | ((m_buff[14] << 5U) & 0x07FF));
            m_sbus.ch[10] = (uint32_t)((m_buff[14] >> 6U) | ((m_buff[15] << 2U) | (m_buff[16] << 10U) & 0x07FF));
            m_sbus.ch[11] = (uint32_t)((m_buff[16] >> 1U) | ((m_buff[17] << 7U) & 0x07FF));
            m_sbus.ch[12] = (uint32_t)((m_buff[17] >> 4U) | ((m_buff[18] << 4U) & 0x07FF));
            m_sbus.ch[13] = (uint32_t)((m_buff[18] >> 7U) | ((m_buff[19] << 1U) | ((m_buff[20] << 9U) & 0x07FF)));
            m_sbus.ch[14] = (uint32_t)((m_buff[20] >> 2U) | ((m_buff[21] << 6U) & 0x07FF));
            m_sbus.ch[15] = (uint32_t)((m_buff[21] >> 5U) | ((m_buff[22] << 3U) & 0x07FF));                               
            m_sbus.ch17 = (m_buff[23] & CH17_MASK) ? true : false;
            m_sbus.ch18 = (m_buff[23] & CH18_MASK) ? true : false;
          }
        }
        //Decode lost frame bit
        m_sbus.lostFrame = (m_buff[23] & LOST_FRAME_MASK) ? true : false;
        //Decode failsafe bit
        m_sbus.failsafe = (m_buff[23] & FAILSAFE_MASK) ? true : false;
        lossOfCommsTimer.set(COMMS_TIME_OUT_PERIOD);
        m_lostComms = false;

        if constexpr(Config::DEBUG_SBUS)
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
    m_lostComms = true;
  }

  return false;
}


/**
* @brief    Returns the most recent SBus data.  
* @return   SbusPacket
*/  
const SBus::SbusPacket 
SBus::getData()
{
  return m_sbus;
}


/**
* @brief    Prints SBus data to console    
* @param    True when Sbus comms has been lost
*/  
const bool
SBus::hasLostCommunications() const
{
  return m_lostComms;
}


/**
* @brief    Prints SBus data to console    
*/  
void 
SBus::printData(void)
{
  uint8_t channels;

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

  // Display the received data 
  for (uint8_t i = 0U; i < channels; i++) 
  {
    Serial.print(m_sbus.ch[i]);
    Serial.print("\t");
  }
  // Display lost frames and failsafe data 
  Serial.print(m_sbus.lostFrame);
  Serial.print("\t");
  Serial.println(m_sbus.failsafe);
}