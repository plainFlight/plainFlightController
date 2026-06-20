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
* @file   CrsfCodec.hpp
* @brief  Stateless CRSF protocol codec: CRC, channel unpacking, and all
*         CRSF wire-format constants required for frame reception.
*
* Design rationale
* ----------------
* This class is deliberately separated from Crsf (the UART transport implementation)
* for two reasons:
*
*   1. Bearer independence: the protocol knowledge can be shared by future bearer
*      implementations (e.g. ESP-NOW) without duplication.  Each bearer handles its
*      own framing/transport; all of them call CrsfCodec for encoding and decoding.
*
*   2. Testability: stateless pure functions are easy to unit-test in isolation,
*      without requiring a live UART or hardware mock.
*
* All methods are static and the class cannot be instantiated (constructor is deleted).
*
*/

#pragma once

#include <inttypes.h>

/**
* @class  CrsfCodec
* @brief  Static utility class for CRSF frame decoding and CRC calculation.
*         Cannot be instantiated; all members are static.
*/
class CrsfCodec
{
  public:

    // -------------------------------------------------------------------------
    // Frame structure constants
    // Byte offsets within any CRSF frame buffer.
    // -------------------------------------------------------------------------

    /** Maximum CRSF frame size in bytes (includes sync byte and CRC). */
    static constexpr uint32_t MAX_FRAME_SIZE          = 64U;

    /** Byte offset of the sync / device-address byte within a frame buffer. */
    static constexpr uint32_t SYNC_OFFSET             = 0U;

    /** Byte offset of the frame-length field within a frame buffer.
     *  The frame-length value = type(1) + payload bytes + CRC(1). */
    static constexpr uint32_t LENGTH_OFFSET           = 1U;

    /** Byte offset of the frame-type byte within a frame buffer. */
    static constexpr uint32_t TYPE_OFFSET             = 2U;

    /** Byte offset of the first payload byte within a frame buffer. */
    static constexpr uint32_t PAYLOAD_OFFSET          = 3U;

    /**
     * Total byte count of a complete CRSF battery sensor frame.
     *   sync(1) + length(1) + type(1) + payload(8) + CRC(1) = 12 bytes.
     *
     * Payload breakdown (8 bytes):
     *   voltage(2) + current(2) + capacity_used(3) + remaining(1)
     */
    static constexpr uint32_t BATTERY_FRAME_SIZE  = 12U;


    // -------------------------------------------------------------------------
    // Device address constants  (CRSF spec: Device Addresses)
    // -------------------------------------------------------------------------

    /** Flight controller device address; also used as the sync byte for frames
     *  sent to / from the FC. */
    static constexpr uint8_t  ADDR_FLIGHT_CONTROLLER  = 0xC8U;

    /** RC receiver device address. */
    static constexpr uint8_t  ADDR_RECEIVER           = 0xECU;


    // -------------------------------------------------------------------------
    // Frame type constants  (CRSF spec: Broadcast Frame Types)
    // -------------------------------------------------------------------------

    /** GPS telemetry frame type (spec section 0x02).
     *  Reserved; frame builder to be added in the GPS telemetry feature PR. */
    static constexpr uint8_t  FRAMETYPE_GPS           = 0x02U;

    /** Battery sensor telemetry frame type (spec section 0x08).
     *  Reserved; frame builder to be added in the battery telemetry feature PR. */
    static constexpr uint8_t  FRAMETYPE_BATTERY       = 0x08U;

    /** Link statistics frame type (spec section 0x14). */
    static constexpr uint8_t  FRAMETYPE_LINK_STATS    = 0x14U;

    /** Packed RC channels frame type (spec section 0x16). */
    static constexpr uint8_t  FRAMETYPE_RC_CHANNELS   = 0x16U;


    // -------------------------------------------------------------------------
    // RC channel constants
    // -------------------------------------------------------------------------

    /** Number of RC channels carried in a single packed channels frame. */
    static constexpr uint32_t NUM_CHANNELS            = 16U;

    /** Raw CRSF channel value at full-low stick position. */
    static constexpr uint32_t MIN_CHANNEL_VALUE       = 172U;

    /** Raw CRSF channel value at centre-stick position. */
    static constexpr uint32_t MID_CHANNEL_VALUE       = 992U;

    /** Raw CRSF channel value at full-high stick position. */
    static constexpr uint32_t MAX_CHANNEL_VALUE       = 1811U;

    /** Bitmask for an 11-bit channel value. */
    static constexpr uint32_t CHANNEL_VALUE_MASK      = 0x07FFU;

    /** Number of payload bytes in a packed RC channels frame.
     *  16 channels x 11 bits = 176 bits = 22 bytes. */
    static constexpr uint32_t RC_PAYLOAD_BYTES        = 22U;


    // -------------------------------------------------------------------------
    // Receive-side frame length guard
    // -------------------------------------------------------------------------

    /**
     * Minimum frame-length field value for a valid link statistics frame.
     * type(1) + 10 payload bytes + CRC(1) = 12.
     * Used by Crsf::parseLinkStatistics() to guard against short frames.
     */
    static constexpr uint32_t LINK_STATS_MIN_FRAME_LENGTH = 12U;


    // -------------------------------------------------------------------------
    // Public static methods
    // -------------------------------------------------------------------------

    /**
    * @brief   Unpack 16 raw 11-bit RC channel values from a CRSF packed-channels payload.
    * @details CRSF packs 16 channels at 11 bits each into 22 bytes, little-endian LSB-first.
    *          The extracted values are raw wire values (range MIN_CHANNEL_VALUE to
    *          MAX_CHANNEL_VALUE); normalisation to the application range is the
    *          caller's responsibility.
    * @param   payload     Pointer to the first byte of the RC channels payload.
    *                      Must point to at least RC_PAYLOAD_BYTES of valid data.
    * @param   rawChannels Output: array of NUM_CHANNELS raw channel values.
    */
    static void unpackChannels(const uint8_t* const payload, uint32_t (&rawChannels)[NUM_CHANNELS]);

    /**
    * @brief   Calculate CRC8 (DVB-S2, polynomial 0xD5) over a data buffer.
    * @details Per the CRSF spec, CRC covers the frame-type byte and all payload
    *          bytes.  It does not include the sync byte or the frame-length byte.
    * @param   data    Pointer to the first byte to include (typically &buffer[TYPE_OFFSET]).
    * @param   length  Number of bytes to process.
    * @return  Calculated CRC8 value.
    */
    static uint8_t calculateCrc(const uint8_t* const data, const uint8_t length);

    /**
    * @brief   Build a complete CRSF Battery Sensor frame (type 0x08) into buf.
    * @details Encodes voltage, current, capacity used, and remaining percentage
    *          from the supplied BatteryData struct into a ready-to-transmit CRSF
    *          frame.  All multi-byte fields are big-endian, matching the CRSF wire
    *          format.  The CRC covers the type byte and all payload bytes, computed
    *          by the existing calculateCrc() helper.
    *
    *          The caller is responsible for supplying a buffer of at least
    *          MAX_FRAME_SIZE bytes.  The returned length will always equal
    *          BATTERY_FRAME_SIZE (12) for a well-formed call.
    *
    * @param   buf   Output buffer; must be at least MAX_FRAME_SIZE bytes.
    * @param   voltageVolts  Battery pack voltage in volts (e.g. 12.6f).
    * @return  Number of bytes written into buf (always BATTERY_FRAME_SIZE).
    */
    static uint8_t buildBatteryFrame(uint8_t* buf, const float voltageVolts);

  private:

    /** Prevent instantiation — this is a static-only utility class. */
    CrsfCodec() = delete;

    /**
    * @brief   Process one byte through the CRC8 DVB-S2 algorithm (polynomial 0xD5).
    * @param   crc  Initial CRC value.
    * @param   a    Byte to fold into the CRC.
    * @return  Updated CRC8 value.
    */
    static uint8_t crc8DvbS2(const uint8_t crc, const uint8_t a);
};