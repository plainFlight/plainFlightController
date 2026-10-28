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
* @file   Srxl2.hpp
* @brief  Spektrum SRXL2 receiver implementation: RC channel data in, no
*         telemetry reply.
*
* Scope
* -----
* This implements just enough of SRXL2 to be recognised as a legitimate bus
* device and reliably receive Channel Data / Failsafe Channel Data packets:
*   - Handshaking: this device announces itself while the bus is quiet (some
*     receivers stay silent on their SRXL2 port until a device speaks) and
*     answers Handshakes addressed to it.
*   - Channel Data / Failsafe Channel Data decoding. Frames with an empty
*     channel mask carry no channel data and are ignored.
* No telemetry reply, no bind pass-through, no VTX handling, and no bus
* master/hub behaviour - this device is always a slave on a single bus
* talking to one receiver. It does not implement ITelemetry; no telemetry is
* sent.
*
* Attribution
* -----------
* Informed by Horizon Hobby's reference implementation (MIT licensed) at
* github.com/SpektrumRC/SRXL2, used here only to confirm wire format and
* state transitions.
*/

#pragma once

#include <inttypes.h>
#include "RxBase.hpp"
#include <HardwareSerial.h>
#include <Arduino.h>
#include "Timer.hpp"


/**
* @class  Srxl2
* @brief  SRXL2 protocol decoder (RC channel input only) derived from RxBase.
*/
class Srxl2 : public RxBase
{
  public:

    /**
    * @brief Channel mapping for SRXL2.
    * @note  SRXL2 channel order (which physical Tx control ends up at which
    *        channel index) is model-memory/programming dependent on the
    *        transmitter; there is no fixed protocol-level convention to
    *        reverse-map. This maps 1:1, and the user assigns RC function
    *        channels in Config.hpp to match their transmitter's channel order.
    */
    static constexpr uint32_t CHANNEL_MAP[18] =
    {
      0U,  // THROTTLE
      1U,  // ROLL
      2U,  // PITCH
      3U,  // YAW
      4U,  // AUX1
      5U,  // AUX2
      6U,  // AUX3
      7U,  // AUX4
      8U,  // AUX5
      9U,  // AUX6
      10U, // AUX7
      11U, // AUX8
      12U, // AUX9
      13U, // AUX10
      14U, // AUX11
      15U, // AUX12
      16U, // AUX13
      17U, // AUX14
    };

    static_assert((sizeof(CHANNEL_MAP) / sizeof(CHANNEL_MAP[0])) ==
      static_cast<uint32_t>(RcChannelName::COUNT),
      "Configuration Error: CHANNEL_MAP size must match RcChannelName::COUNT.");

    // -------------------------------------------------------------------------
    // Wire-format constants and codec methods.
    // -------------------------------------------------------------------------

    /** Sync byte that starts every SRXL2 packet. */
    static constexpr uint8_t  SRXL_SYNC_BYTE          = 0xA6U;

    /** Packet-type byte: Handshake (device discovery/introduction). */
    static constexpr uint8_t  PACKET_TYPE_HANDSHAKE   = 0x21U;

    /** Packet-type byte: Control Data (carries Channel Data / Failsafe Channel Data). */
    static constexpr uint8_t  PACKET_TYPE_CONTROL     = 0xCDU;

    /** Control Data sub-command: normal Channel Data. */
    static constexpr uint8_t  CTRL_CMD_CHANNEL        = 0x00U;

    /** Control Data sub-command: Failsafe Channel Data. */
    static constexpr uint8_t  CTRL_CMD_CHANNEL_FS     = 0x01U;

    /** Byte offset of the sync byte within a frame buffer. */
    static constexpr uint32_t SYNC_OFFSET             = 0U;

    /** Byte offset of the packet-type byte within a frame buffer. */
    static constexpr uint32_t TYPE_OFFSET             = 1U;

    /**
    * Byte offset of the frame-length byte within a frame buffer.
    * The length field counts the whole frame - sync byte through the final
    * CRC byte inclusive.
    */
    static constexpr uint32_t LENGTH_OFFSET           = 2U;

    /** Byte offset of the first payload byte within a frame buffer. */
    static constexpr uint32_t PAYLOAD_OFFSET          = 3U;

    /** Number of trailing CRC bytes on every SRXL2 packet. */
    static constexpr uint32_t CRC_LENGTH              = 2U;

    /** Minimum plausible total frame length (header + CRC, no payload). */
    static constexpr uint32_t MIN_FRAME_LENGTH        = 5U;

    /** Maximum total frame length a buffer must accommodate. */
    static constexpr uint32_t MAX_FRAME_SIZE          = 80U;

    // --- Handshake packet layout ---
    // [0]     sync (0xA6)
    // [1]     packet type (0x21)
    // [2]     length (always 14 for a Handshake packet)
    // [3]     srcDevID
    // [4]     destDevID
    // [5]     priority
    // [6]     baudSupported
    // [7]     info
    // [8-11]  uid, little-endian uint32
    // [12-13] CRC-16, big-endian

    /** Total byte count of a complete Handshake packet. */
    static constexpr uint32_t HANDSHAKE_FRAME_LENGTH      = 14U;
    static constexpr uint32_t HANDSHAKE_SRC_OFFSET         = 3U;
    static constexpr uint32_t HANDSHAKE_DEST_OFFSET        = 4U;
    static constexpr uint32_t HANDSHAKE_PRIORITY_OFFSET    = 5U;
    static constexpr uint32_t HANDSHAKE_BAUD_OFFSET        = 6U;
    static constexpr uint32_t HANDSHAKE_INFO_OFFSET        = 7U;
    static constexpr uint32_t HANDSHAKE_UID_OFFSET         = 8U;

    /** Flight-controller device ID to advertise (FCs occupy the 0x30-0x3F range). */
    static constexpr uint8_t  OUR_DEVICE_ID           = 0x30U;

    /**
    * Telemetry priority to advertise.
    */
    static constexpr uint8_t  OUR_PRIORITY            = 0x00U;

    /** baudSupported bitmask: 0x00 = 115200 only; 400000 is not negotiated. */
    static constexpr uint8_t  BAUD_SUPPORTED_115200_ONLY = 0x00U;

    /** info byte: no RF, no telemetry tx, no forward-programming support - this device is receive-only. */
    static constexpr uint8_t  INFO_NO_TELEMETRY       = 0x00U;

    // --- Control Data packet layout (Channel Data / Failsafe Channel Data) ---
    // [0]      sync (0xA6)
    // [1]      packet type (0xCD)
    // [2]      length (12 + 2*activeChannels + 2)
    // [3]      cmd (CTRL_CMD_CHANNEL or CTRL_CMD_CHANNEL_FS)
    // [4]      replyID (ignored - this device sends no telemetry)
    // [5]      rssi, signed 8-bit
    // [6-7]    frameLosses, little-endian uint16
    // [8-11]   channel mask, little-endian uint32 (bit n set = channel n present)
    // [12...]  channel values, little-endian uint16 each, one per set mask bit,
    //          in ascending bit-index order
    // [last-2..last-1] CRC-16, big-endian

    static constexpr uint32_t CONTROL_CMD_OFFSET       = 3U;
    static constexpr uint32_t CONTROL_REPLY_ID_OFFSET  = 4U;
    static constexpr uint32_t CONTROL_RSSI_OFFSET      = 5U;
    static constexpr uint32_t CONTROL_FRAME_LOSSES_OFFSET = 6U;
    static constexpr uint32_t CONTROL_MASK_OFFSET      = 8U;
    static constexpr uint32_t CONTROL_VALUES_OFFSET    = 12U;

    /** Bytes before the first channel-value byte: header(3) + cmd/replyID(2) + rssi(1) + frameLosses(2) + mask(4). */
    static constexpr uint32_t CONTROL_HEADER_LENGTH    = CONTROL_VALUES_OFFSET;

    /** SRXL2 supports up to 32 channel slots (one per mask bit), regardless of RxBase::MAX_RX_CHANNELS. */
    static constexpr uint32_t MAX_SRXL_CHANNELS        = 32U;

    /**
    * Raw channel value range. A channel value is a 16-bit field centred at
    * MID_CHANNEL_VALUE (0x8000 = "servo center" per spec); the lowest 2 bits
    * are reserved. Spektrum's 11-bit (0-2047) channel value is shifted left 5
    * into this field; 100% travel is 342-1706, so MIN/MAX are those values
    * << 5. Travel beyond 100% is constrained to these limits.
    */
    static constexpr uint32_t MIN_CHANNEL_VALUE        = 10944U;   // 342 << 5
    static constexpr uint32_t MID_CHANNEL_VALUE        = 32768U;   // 1024 << 5
    static constexpr uint32_t MAX_CHANNEL_VALUE        = 54592U;   // 1706 << 5

    /**
    * @brief   Calculate CRC-16/XMODEM (poly 0x1021, no reflection) over a data buffer.
    * @param   data    Pointer to the first byte to include.
    * @param   length  Number of bytes to process.
    * @return  Calculated CRC-16 value.
    * @note    Covers the whole frame excluding its own trailing CRC bytes -
    *          that includes the sync byte and length byte.
    */
    static uint16_t calculateCrc(const uint8_t* data, uint32_t length);

    /**
    * @brief   Validate a complete frame buffer's CRC-16.
    * @param   buffer  Pointer to the frame buffer, starting at the sync byte.
    * @param   length  Total frame length (the value of buffer[LENGTH_OFFSET]).
    * @return  true if length is at least MIN_FRAME_LENGTH and the trailing
    *          big-endian CRC-16 matches calculateCrc() over the rest of the frame.
    */
    static bool isFrameCrcValid(const uint8_t* buffer, uint32_t length);

    /**
    * @brief   Count how many of the 32 possible channel-mask bits are set.
    * @param   mask  A Control Data packet's channel mask field.
    * @return  Number of active (present) channels in this frame.
    */
    static uint32_t countActiveChannels(uint32_t mask);

    /**
    * @brief   Read a little-endian 16-bit value from a buffer.
    */
    static uint32_t readLE16(const uint8_t* p);

    /**
    * @brief   Read a little-endian 32-bit value from a buffer.
    */
    static uint32_t readLE32(const uint8_t* p);

    /**
    * @brief   Write a 32-bit value into a buffer, little-endian.
    */
    static void writeLE32(uint8_t* p, uint32_t value);

    /**
    * @brief   Unpack raw channel values from a Control Data packet's channel-value region.
    * @details Only entries whose mask bit is set are written; other slots of
    *          rawValues are left untouched. Values are raw wire values (see
    *          MIN_CHANNEL_VALUE/MAX_CHANNEL_VALUE); normalisation to the
    *          application range is the caller's responsibility.
    * @param   valuesStart  Pointer to the first channel-value byte (i.e.
    *                       &buffer[CONTROL_VALUES_OFFSET]).
    * @param   mask         The frame's channel mask field.
    * @param   rawValues    Output: raw values indexed by channel/bit number (0-31).
    * @return  Number of channels unpacked (== countActiveChannels(mask)).
    */
    static uint32_t unpackChannelValues(const uint8_t* valuesStart, uint32_t mask,
                                         uint32_t (&rawValues)[MAX_SRXL_CHANNELS]);

    /**
    * @brief   Compute the total frame length a Control Data packet with the
    *          given channel mask should have.
    * @param   mask  The frame's channel mask field.
    * @return  CONTROL_HEADER_LENGTH + 2 bytes per active channel + CRC_LENGTH.
    */
    static uint32_t controlFrameExpectedLength(uint32_t mask);

    /**
    * @brief   Build a complete Handshake packet into buf.
    * @param   buf        Output buffer; must be at least HANDSHAKE_FRAME_LENGTH bytes.
    * @param   destDevId  Device ID to address the handshake to: the srcDevID of
    *                     a Handshake being answered, or UNKNOWN_MASTER_ID for
    *                     an unprompted one.
    * @param   uid        32-bit value to advertise as this device's UID.
    * @return  Number of bytes written (always HANDSHAKE_FRAME_LENGTH).
    */
    static uint32_t buildHandshakeFrame(uint8_t* buf, uint8_t destDevId, uint32_t uid);

    // -------------------------------------------------------------------------
    // Transport: UART ownership, byte-stream state machine, RxBase overrides.
    // -------------------------------------------------------------------------

    /**
    * @brief   Constructor for SRXL2 receiver.
    * @details Resolves its UART and pins from Config::RECEIVER_SERIAL_PORT via
    *          InternalConfig, initialises the UART at 115200 baud, and flushes
    *          any stale bytes so getDemands() begins with a clean stream.
    * @note    SRXL2 is a single-wire, half-duplex bus: the receiver and this
    *          device share one signal line, which idles high.
    *          Wiring: connect the receiver's signal line directly to the RX
    *          pin of Config::RECEIVER_SERIAL_PORT, and to its TX pin through
    *          a Schottky diode (e.g. BAT85, 1N5817) with the cathode (band)
    *          towards the TX pin. TX can then only pull the line low and
    *          never fights the receiver. Do not join RX and TX with a plain
    *          wire or set them to the same GPIO: the UART's TX output is
    *          push-pull and drives the line high while idle.
    *          Pull-up: Spektrum receivers normally pull the line up
    *          themselves (about 3.3V on the unconnected signal pin). If the
    *          line floats, add about 4.7k from the signal line to 3V3. Never
    *          pull up to 5V: this GPIO's absolute maximum is 3.6V.
    *          This device's own transmitted bytes echo back into its RX; the
    *          parser ignores its own handshakes, and resyncs on the next
    *          valid sync+length+CRC after any stray bytes. On boards whose
    *          receiver port is on GPIO43/44 (UART0), the ESP32-S3 boot
    *          messages also go out on TX at power-up; they are not valid
    *          SRXL2 frames.
    *          Startup: It is possible some receivers only switch their port to 
    *          SRXL2 if a device speaks shortly after they power up. If no 
    *          channel data arrives, power-cycle the receiver with the transmitter 
    *          off, then switch the transmitter on.
    */
    Srxl2();

    /**
    * @brief   Get new data from the SRXL2 receiver.
    * @return  true when new data is available, false otherwise.
    */
    bool getDemands() override;

    /**
    * @brief   Print receiver data to console for debugging.
    */
    void printData();

    /**
    * @brief   Get receiver data packet.
    * @return  RxPacket containing failsafe status, communication status, and normalised channel data.
    */
    const RxPacket getData() override;

    /**
    * @brief   Check if communications have been lost.
    * @return  true if communications lost, false otherwise.
    */
    const bool hasLostCommunications() override;

    /**
    * @brief   Link quality from the RSSI byte of the last channel data packet.
    * @note    SRXL2 sends a positive value as a percentage and a negative value as dBm;
    * @note    ...only the percentage form is reported.
    * @param   percent  Set to the link quality, 0 to 100, when this returns true; unchanged otherwise.
    * @return  true if the last channel data packet carried a percentage value,
    *          false if it carried dBm or no packet has been received.
    */
    bool getLinkQuality(uint8_t& percent) const override
    {
      bool available = false;
      if (m_rssi >= 0)
      {
        percent = static_cast<uint8_t>(m_rssi);
        available = true;
      }
      return available;
    }

    /**
    * @brief   Get channel index from channel name.
    * @param   name  The channel name enum.
    * @return  Channel index (0-based).
    */
    uint32_t getChannelIndex(RcChannelName name) const override
    {
      return CHANNEL_MAP[static_cast<uint32_t>(name)];
    }

  private:
    static constexpr uint8_t  UNKNOWN_MASTER_ID              = 0x00U;  // Destination of an unprompted handshake.
    static constexpr uint32_t UNPROMPTED_HANDSHAKE_PERIOD_MS = 50U;    // Bus silence before, and interval between, unprompted handshakes.

    uint32_t m_lastBusActivityMs   = 0U;   // millis() of the last valid frame from another device.
    uint32_t m_lastHandshakeSentMs = 0U;   // millis() of our last unprompted handshake.

    static constexpr uint32_t SRXL2_BAUD            = 115200U;
    static constexpr uint32_t MAX_BYTES_PER_LOOP     = 12U;   // Bytes consumed per getDemands() call (115200 baud delivers up to 11.5 bytes/ms), bounded so the main loop is never blocked.
    static constexpr uint64_t COMMS_TIME_OUT_PERIOD  = 100U;

    HardwareSerial* m_uart;
    uint32_t m_bufferIndex   = 0U;
    uint8_t  m_buffer[MAX_FRAME_SIZE] = {};
    int8_t   m_rssi          = -1;   // RSSI byte of the last channel data packet (see getLinkQuality()).

    // Objects
    CTimer m_lossOfCommsTimer = CTimer(0);

    /**
    * @brief   Send a complete, CRC-valid frame by packet type.
    * @param   buffer  The frame buffer, starting at the sync byte.
    * @param   length  Total frame length.
    * @return  true if this frame updated m_rxData with new channel data.
    */
    bool processFrame(const uint8_t* buffer, uint32_t length);

    /**
    * @brief   Decode a Channel Data / Failsafe Channel Data Control Data frame.
    * @param   buffer  The frame buffer, starting at the sync byte.
    * @param   length  Total frame length.
    * @return  true if m_rxData was updated with new channel data.
    */
    bool handleControlData(const uint8_t* buffer, uint32_t length);

    /**
    * @brief   Handle a Handshake frame witnessed on the bus, replying if it is
    *          addressed to this device.
    * @param   buffer  The frame buffer, starting at the sync byte.
    * @param   length  Total frame length.
    */
    void handleHandshake(const uint8_t* buffer, uint32_t length);

    /**
    * @brief   Build and transmit our own Handshake packet.
    * @param   destDevId  Device ID to address the handshake to.
    */
    void sendHandshake(uint8_t destDevId);

    /**
    * @brief   A statistically-unique 32-bit value to advertise as this device's UID.
    * @return  The low 32 bits of the ESP32's factory-programmed MAC address.
    */
    static uint32_t deviceUid();
};
