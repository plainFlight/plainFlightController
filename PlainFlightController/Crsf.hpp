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
* @file   Crsf.hpp
* @brief  This class handles communications with a CRSF/ELRS RC receiver.
*/

#pragma once

#include <inttypes.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "Timer.hpp"
#include "Config.hpp"
#include "RxBase.hpp"


/**
 * @class Crsf
 * @brief CRSF protocol decoder implementation derived from RxBase.
 */
class Crsf : public RxBase
{
  public:
    static constexpr uint32_t MIN_CRSF_US        = 172U;
    static constexpr uint32_t MID_CRSF_US        = 992U;
    static constexpr uint32_t MAX_CRSF_US        = 1811U;

    /**
     * @brief Channel mapping for CRSF protocol (AETR ordering).
     * Maps standard channel names to physical CRSF channel positions.
     * CRSF native ordering: Aileron, Elevator, Throttle, Rudder
     */
    static constexpr uint32_t CHANNEL_MAP[9] = 
    {
      2U,  // THROTTLE
      0U,  // ROLL (Aileron)
      1U,  // PITCH (Elevator)
      3U,  // YAW (Rudder)
      4U,  // ARM
      5U,  // MODE
      6U,  // AUX1
      7U,  // AUX2
      8U   // AUX3
    };

    /**
     * @struct CrsfLinkStats
     * @brief CRSF-specific link quality statistics.
     */
    struct CrsfLinkStats
    {
      uint8_t linkQuality;  // Link quality percentage (0-100)
      int8_t rssiDbm;       // RSSI in dBm (-128 to 0)
      uint8_t txPower;      // TX power index
      int8_t snrDb;         // SNR in dB
    };
    
    /**
     * @brief Constructor for CRSF receiver.
     * @param uart Pointer to hardware serial port.
     * @param rxPin RX pin number.
     * @param txPin TX pin number.
     */
    Crsf(HardwareSerial *uart, uint8_t rxPin, uint8_t txPin);

    /**
     * @brief Get new data from CRSF receiver.
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
    uint32_t getChannelIndex(ChannelName name) const override
    {
      return CHANNEL_MAP[static_cast<uint32_t>(name)];
    }

    /**
     * @brief Get CRSF-specific link statistics.
     * @return CrsfLinkStats structure containing link quality data.
     */
    CrsfLinkStats getLinkStats() const;

  private:
    static constexpr uint32_t CRSF_BAUD                 = 420000U;
    static constexpr uint32_t CRSF_MAX_FRAME_SIZE       = 64U;
    static constexpr uint32_t CRSF_PAYLOAD_OFFSET       = 3U;
    static constexpr uint32_t CRSF_FRAMETYPE_OFFSET     = 2U;
    static constexpr uint32_t CRSF_LENGTH_OFFSET        = 1U;
    static constexpr uint32_t NUM_CRSF_CH               = 16U;
    static constexpr uint32_t MAX_BYTES_PER_LOOP        = 6U;
    static constexpr uint32_t CRSF_RC_FRAME_SIZE        = 22U;
    static constexpr uint32_t CRSF_LINK_FRAME_SIZE      = 12U;
    static constexpr uint64_t COMMS_TIME_OUT_PERIOD     = 100U;
    static constexpr uint8_t  FAILSAFE_LQ_THRESHOLD     = 20U;
    
    // CRSF Device Addresses
    static constexpr uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8U;
    static constexpr uint8_t CRSF_ADDRESS_ALTERNATIVE       = 0xECU;
    
    // CRSF Frame Types
    static constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS = 0x16U;
    static constexpr uint8_t CRSF_FRAMETYPE_LINK_STATS  = 0x14U;

    HardwareSerial *m_uart;
    uint8_t m_bufferIndex = 0U;
    uint8_t m_buffer[CRSF_MAX_FRAME_SIZE] = {};
    CrsfLinkStats m_crsfLinkStats = {0U, -128, 0U, -128};

    CTimer lossOfCommsTimer = CTimer(0);

    /**
     * @brief Parse RC channels from CRSF payload.
     * @param payload Pointer to RC channels payload (22 bytes).
     * @param payloadLength Length of payload.
     */
    void parseRcChannels(const uint8_t *payload, uint8_t payloadLength);

    /**
     * @brief Parse link statistics from CRSF payload.
     * @param payload Pointer to link statistics payload.
     * @param payloadLength Length of payload.
     */
    void parseLinkStatistics(const uint8_t *payload, uint8_t payloadLength);

    /**
     * @brief Calculate CRC8 for a single byte (DVB-S2 polynomial).
     * @param crc Current CRC value.
     * @param a Byte to process.
     * @return Updated CRC8 value.
     */
    uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) const;

    /**
     * @brief Calculate CRC8 for a data buffer.
     * @param data Pointer to data buffer.
     * @param length Length of data.
     * @return CRC8 value.
     */
    uint8_t calculateCrc(const uint8_t *data, uint8_t length) const;
};
