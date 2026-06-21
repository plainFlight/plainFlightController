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
* @file   Crsf.hpp
* @brief  CRSF/ELRS receiver implementation.
*
* This class handles the CRSF/ELRS receive path: it decodes the CRSF byte
* stream from the receiver into normalised RC channel data, accessible via
* the RxBase interface.
*
* All protocol encoding/decoding and wire-format constants live in CrsfCodec.
* This class owns only the UART transport, the receive state machine, and
* the application-level policy constants (timeouts, thresholds).
*
* Telemetry transmit capability
* ------------------------------
* Telemetry transmission (ITelemetry interface) will be added in a subsequent
* PR alongside the first concrete telemetry data source.  At that point Crsf
* will inherit from both RxBase and ITelemetry, and the flight controller will
* hold a second pointer of type ITelemetry* alongside its existing RxBase*.
*/

#pragma once

#include <inttypes.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "Timer.hpp"
#include "Config.hpp"
#include "RxBase.hpp"
#include "CrsfCodec.hpp"
#include "ITelemetry.hpp"


/**
* @class  Crsf
* @brief  CRSF/ELRS protocol implementation providing RC channel input via RxBase.
*
* Protocol knowledge (frame format, CRC, channel packing) is delegated entirely
* to CrsfCodec; this class owns only transport and application state.
*/
class Crsf : public RxBase, public ITelemetry
{
  public:

    /**
    * @brief Channel mapping for CRSF/ELRS protocol (AETR ordering).
    *
    * Maps logical ChannelName values to their physical CRSF channel indices.
    * CRSF transmits channels in Aileron(0), Elevator(1), Throttle(2), Rudder(3)
    * order; this table re-orders them to the flight controller's convention.
    */
    static constexpr uint32_t CHANNEL_MAP[9] =
    {
      2U,   // THROTTLE → CRSF channel 2
      0U,   // ROLL     → CRSF channel 0 (Aileron)
      1U,   // PITCH    → CRSF channel 1 (Elevator)
      3U,   // YAW      → CRSF channel 3 (Rudder)
      4U,   // ARM      → CRSF channel 4
      5U,   // MODE     → CRSF channel 5
      6U,   // AUX1     → CRSF channel 6
      7U,   // AUX2     → CRSF channel 7
      8U    // AUX3     → CRSF channel 8
    };

    /**
    * @struct CrsfLinkStats
    * @brief  CRSF uplink quality statistics decoded from frame type 0x14.
    *
    * Updated each time a valid link statistics frame is received.
    * linkQuality is also used by parseRcChannels() for multi-level failsafe
    * detection — if it falls below FAILSAFE_LQ_THRESHOLD the failsafe flag
    * is asserted even if frames are still arriving.
    */
    struct CrsfLinkStats
    {
      uint8_t linkQuality;   ///< Uplink packet success rate, percent (0–100)
      int8_t  rssiDbm;       ///< Uplink RSSI in dBm (negative; closer to 0 = stronger)
      uint8_t txPower;       ///< Transmitter power index (see CRSF spec 0x14)
      int8_t  snrDb;         ///< Uplink signal-to-noise ratio in dB
    };

    /**
    * @brief   Constructor.
    * @details Initialises the UART at CRSF_BAUD (420000 baud for ELRS) and
    *          flushes any stale bytes before the first getDemands() call.
    * @param   uart    Pointer to the HardwareSerial port to use.
    * @param   rxPin   GPIO pin number connected to the receiver's TX line.
    * @param   txPin   GPIO pin number connected to the receiver's RX line.
    */
    Crsf(HardwareSerial* const uart, const uint8_t rxPin, const uint8_t txPin);

    /**
    * @brief   Read and decode incoming CRSF stream bytes.
    * @details Implements a three-state byte-stream parser:
    *            State 0 — hunt for a valid sync byte.
    *            State 1 — read and validate the frame-length byte.
    *            State 2+ — accumulate bytes until the complete frame is buffered,
    *                       validate CRC, then dispatch to the appropriate parser.
    *          At most MAX_BYTES_PER_LOOP bytes are consumed per call so that
    *          the main loop is never blocked for longer than a bounded time.
    * @return  true when a new, valid RC channels frame has been decoded and
    *          m_rxData updated; false on all other calls.
    */
    bool getDemands() override;

    /**
    * @brief Prints channel data and status flags to Serial for debugging.
    * @note  Rate-limited internally; active only when InternalConfig::DEBUG_RX
    *        is true (checked by the caller via if constexpr).
    */
    void printData();

    /**
    * @brief   Return the most recently decoded RC data packet.
    * @return  RxPacket with failsafe flag, lostComms flag, and 16 normalised
    *          channel values in the range [MIN_NORMALISED .. MAX_NORMALISED].
    */
    const RxPacket getData() override;

    /**
    * @brief   Report whether CRSF communications have been lost.
    * @return  true if no valid RC channels frame has been received within
    *          COMMS_TIME_OUT_PERIOD milliseconds.
    */
    const bool hasLostCommunications() override;

    /**
    * @brief   Resolve a logical ChannelName to its physical CRSF channel index.
    * @param   name  Logical channel name enum value.
    * @return  Physical channel index (0-based) for use with RxPacket::ch[].
    */
    uint32_t getChannelIndex(ChannelName name) const override
    {
      return CHANNEL_MAP[static_cast<uint32_t>(name)];
    }

    /**
    * @brief   Return the most recently received CRSF link quality statistics.
    * @return  CrsfLinkStats populated from the last valid 0x14 frame.
    *          Values are initialised to worst-case on construction.
    */
    CrsfLinkStats getLinkStats() const;

    /**
    * @brief   Send battery voltage telemetry to the RC transmitter.
    * @details Delegates frame assembly entirely to CrsfCodec::buildBatteryFrame(),
    *          which handles the unit conversion from volts to the CRSF wire format
    *          and packs all other fields.  The resulting bytes are written directly
    *          to the CRSF UART.  Rate limiting is the caller's responsibility.
    * @param   voltageVolts  Battery pack voltage in volts (e.g. 12.6f).
    */
    void sendBatteryTelemetry(const float& voltageVolts) override;

    /**
    * @brief   Send gps data packet telemetry to the RC transmitter.
    * @details Delegates frame assembly entirely to CrsfCodec::buildGPSFrame(),
    *          which handles the unit conversion and packs all other fields.  
    *          The resulting bytes are written directly to the CRSF UART.  
    *          Rate limiting is the caller's responsibility.
    * @param   data  A GnssData structure.
    */
    void sendGnssTelemetry(const GnssData& data) override;

  private:

    // -------------------------------------------------------------------------
    // Transport and application-policy constants.
    // Protocol wire-format constants (frame structure, addresses, frame types,
    // channel ranges) have moved to CrsfCodec and are accessed via CrsfCodec::.
    // -------------------------------------------------------------------------

    /**
    * UART baud rate for CRSF/ELRS.
    * ExpressLRS uses 420000 baud.  The original CRSF specification states
    * 416666 baud for the flying-platform UART; 420000 is correct for ELRS.
    */
    static constexpr uint32_t CRSF_BAUD             = 420000U;

    /**
    * Maximum number of bytes consumed from the UART per getDemands() call.
    * Caps the worst-case time spent in getDemands() to keep the main loop
    * bounded.  At 420000 baud, 10 bytes takes approximately 190 µs.
    */
    static constexpr uint32_t MAX_BYTES_PER_LOOP    = 10U;

    /**
    * Time in milliseconds after which, if no valid RC channels frame has been
    * received, lostComms is set true and failsafe is asserted.
    */
    static constexpr uint64_t COMMS_TIME_OUT_PERIOD = 100U;

    /**
    * Link quality percentage below which the failsafe flag is asserted even
    * if frames are still arriving.  Provides an early-warning failsafe before
    * total signal loss occurs.
    */
    static constexpr uint8_t  FAILSAFE_LQ_THRESHOLD = 20U;


    // -------------------------------------------------------------------------
    // Member variables
    // -------------------------------------------------------------------------

    // Variables
    /** Pointer to the underlying HardwareSerial peripheral. */
    HardwareSerial* m_uart;

    /** Index of the next byte to write into m_buffer; also encodes parser state. */
    uint8_t         m_bufferIndex                            = 0U;

    /** Receive frame assembly buffer.  Sized to the maximum CRSF frame length. */
    uint8_t         m_buffer[CrsfCodec::MAX_FRAME_SIZE]      = {};

    /** Most recently received link statistics; initialised to worst-case values. */
    CrsfLinkStats   m_crsfLinkStats                          = {0U, -128, 0U, -128};

    /**
    * Reusable output buffer for outgoing telemetry frames.
    * Sized to the maximum CRSF frame length so that any frame type can be built 
    * without a separate buffer.
    */
    uint8_t m_txFrameBuffer[CrsfCodec::MAX_FRAME_SIZE] = {};

    // Objects
    /** Timer used to detect loss of communications. */
    CTimer          m_lossOfCommsTimer                         = CTimer(0);


    // -------------------------------------------------------------------------
    // Private methods
    // -------------------------------------------------------------------------

    /**
    * @brief   Parse a CRSF RC channels payload and update m_rxData.
    * @details Validates that frameLength indicates a full 22-byte payload,
    *          delegates 11-bit channel unpacking to CrsfCodec::unpackChannels(),
    *          normalises each raw value to [MIN_NORMALISED .. MAX_NORMALISED]
    *          via map32(), and updates the multi-level failsafe flag.
    * @param   payload      Pointer to the first payload byte in m_buffer.
    * @param   frameLength  The CRSF frame-length field value
    *                       (= type byte + payload bytes + CRC byte).
    */
    void parseRcChannels(const uint8_t* payload, uint8_t frameLength);

    /**
    * @brief   Parse a CRSF link statistics payload and update m_crsfLinkStats.
    * @details Validates that frameLength indicates a sufficiently long payload,
    *          then reads RSSI, link quality, SNR, and TX power from the payload
    *          bytes at the offsets defined by the CRSF 0x14 frame specification.
    * @param   payload      Pointer to the first payload byte in m_buffer.
    * @param   frameLength  The CRSF frame-length field value.
    */
    void parseLinkStatistics(const uint8_t* payload, uint8_t frameLength);
};