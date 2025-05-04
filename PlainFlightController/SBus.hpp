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
* @file   Sbus.hpp
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
#pragma once

#include <inttypes.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "Timer.hpp"
#include "Config.hpp"



/**
 * @class SBus
 */
 
class SBus
{
  public:
    static constexpr uint32_t NUMBER_RX_CH          = 16U;
    static constexpr uint32_t MIN_SBUS_US           = 172U;
    static constexpr uint32_t MID_SBUS_US           = 991U;
    static constexpr uint32_t MAX_SBUS_US           = 1810U;
    static constexpr int32_t  MIN_NORMALISED_US     = -819;
    static constexpr int32_t  MID_NORMALISED_US     = 0;
    static constexpr int32_t  MAX_NORMALISED_US     = 819;
    static constexpr uint32_t SWITCH_HIGH_SBUS_US   = 1391U;
    static constexpr uint32_t SWITCH_LOW_SBUS_US    = 591U;
    static constexpr uint32_t LOW_THROTTLE_SBUS_US  = 250;
    static constexpr bool USE_ALL_18_CHANNELS       = false;

    struct SbusPacket
    {
      bool lostFrame;
      bool failsafe;
      bool ch17;
      bool ch18;
      uint32_t ch[NUMBER_RX_CH];
    };

    enum class ChannelMap : uint32_t
    {
      THROTTLE = 0U,
      ROLL,
      PITCH,
      YAW,
      ARM,
      MODE,
      AUX1,
      AUX2,
      AUX3
    };
    
    SBus(HardwareSerial *uart, uint8_t rxPin, uint8_t txPin);
    bool getDemands();  
    void printData();
    const SbusPacket getData();
    const bool hasLostCommunications() const;

  private:
    static constexpr uint32_t SBUS_BAUD       = 100000U;
    static constexpr uint32_t PAYLOAD_LEN     = 23U;
    static constexpr uint32_t HEADER_LEN      = 1U;
    static constexpr uint32_t FOOTER_LEN      = 1U;
    static constexpr uint32_t NUM_SBUS_CH     = 16;
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
    SbusPacket m_sbus = {true, true, false, false, {0U}};
    uint32_t m_count = 0U;
    uint32_t m_prevByte = FOOTER;
    uint32_t m_buff[BUFFER_SIZE] = {};
    bool m_lostComms = false;

    //Objects
    CTimer lossOfCommsTimer = CTimer(0);
};