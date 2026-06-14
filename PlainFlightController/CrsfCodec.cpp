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
* @file   CrsfCodec.cpp
* @brief  CRSF protocol codec implementation: CRC calculation and RC channel unpacking.
*
* Implementation notes
* --------------------
* CRC
*   All CRSF frames use CRC8 with the DVB-S2 polynomial (0xD5).  The CRC
*   covers the frame-type byte and all payload bytes; it excludes the sync
*   byte and the frame-length byte.  calculateCrc() and crc8DvbS2() implement
*   this for use by both the receive-side CRC validator in Crsf::getDemands()
*   and, in future, the transmit-side frame builders.
*
* MISRA alignment
*   Explicit static_cast<> is used throughout in preference to relying on
*   implicit arithmetic promotions.  In particular:
*     - uint8_t operands are cast to uint32_t before shift operations to
*       avoid the implicit promotion to signed int that the C++ standard
*       applies to sub-int types.
*     - Results of arithmetic on wider types are explicitly narrowed back to
*       uint8_t when stored into byte-array slots or uint8_t variables.
*
* Telemetry frame builders
* ------------------------
* buildGpsFrame(), buildBatteryFrame() and their supporting helpers
* (writeFrameHeader, finaliseFrame) are intentionally absent from this PR.
* They will be added by dedicated feature PRs for each telemetry data source.
*/

#include "CrsfCodec.hpp"


// =============================================================================
// CRC functions
// =============================================================================

/**
* @brief   Process one byte through the CRC8 DVB-S2 algorithm (polynomial 0xD5).
* @details The algorithm XORs the input byte into the running CRC and then
*          processes each bit, conditionally applying the 0xD5 polynomial on
*          each '1' bit shifted out of the MSB.
* @param   crc  Running CRC accumulator (0x00 on first call).
* @param   a    Byte to fold into the CRC.
* @return  Updated CRC8 value.
*/
uint8_t
CrsfCodec::crc8DvbS2(uint8_t crc, uint8_t a)
{
  crc ^= a;
  for (uint32_t i = 0U; i < 8U; i++)
  {
    if ((crc & 0x80U) != 0U)
    {
      // Cast to uint32_t before shifting to avoid signed-int promotion of uint8_t.
      crc = static_cast<uint8_t>((static_cast<uint32_t>(crc) << 1U) ^ 0xD5U);
    }
    else
    {
      crc = static_cast<uint8_t>(static_cast<uint32_t>(crc) << 1U);
    }
  }
  return crc;
}


/**
* @brief   Calculate CRC8 (DVB-S2, polynomial 0xD5) over a data buffer.
* @details Iterates over each byte in the buffer, folding it into the CRC
*          accumulator via crc8DvbS2().
* @param   data    Pointer to the first byte to include (must not be nullptr).
* @param   length  Number of bytes to process.
* @return  Final CRC8 value.
*/
uint8_t
CrsfCodec::calculateCrc(const uint8_t* data, uint8_t length)
{
  uint8_t crc = 0U;
  for (uint32_t i = 0U; i < static_cast<uint32_t>(length); i++)
  {
    crc = crc8DvbS2(crc, data[i]);
  }
  return crc;
}


// =============================================================================
// Channel unpacking
// =============================================================================

/**
* @brief   Unpack 16 raw 11-bit RC channel values from a CRSF packed-channels payload.
* @details CRSF packs 16 channels at 11 bits each into a 22-byte little-endian
*          bitstream (176 bits total).  Each channel value occupies 11 consecutive
*          bits starting at bit offset (channel_index * 11) counting from bit 0 of
*          payload[0].
*
*          The bit manipulation for each channel is:
*            channel_n = bits[(n*11) .. (n*11 + 10)]
*
*          Worked example for channel 2 (bits 22-32):
*            bit 22 = payload[2] bit 6  -> payload[2] >> 6  (2 bits)
*            bit 24 = payload[3] bit 0  -> payload[3] << 2  (8 bits, shifted up 2)
*            bit 32 = payload[4] bit 0  -> payload[4] << 10 (1 bit,  shifted up 10)
*            result = ((p[2]>>6) | (p[3]<<2) | (p[4]<<10)) & 0x7FF
*
*          All payload bytes are cast to uint32_t before shifting to avoid the
*          implicit promotion to signed int that C++ applies to uint8_t operands.
*
*          No length validation is performed here; the caller (Crsf::parseRcChannels)
*          must verify that the payload is at least RC_PAYLOAD_BYTES long before
*          calling this function.
*
* @param   payload     Pointer to the first byte of the RC channels payload.
* @param   rawChannels Output array; receives 16 raw values in range
*                      [MIN_CHANNEL_VALUE .. MAX_CHANNEL_VALUE].
*/
void
CrsfCodec::unpackChannels(const uint8_t* payload, uint32_t (&rawChannels)[NUM_CHANNELS])
{
  // Channels 0-7 (payload bytes 0-10)
  rawChannels[0]  =  (static_cast<uint32_t>(payload[0])
                   | (static_cast<uint32_t>(payload[1])  << 8U))
                   & 0x07FFU;

  rawChannels[1]  =  ((static_cast<uint32_t>(payload[1])  >> 3U)
                   |  (static_cast<uint32_t>(payload[2])  << 5U))
                   & 0x07FFU;

  rawChannels[2]  =  ((static_cast<uint32_t>(payload[2])  >> 6U)
                   |  (static_cast<uint32_t>(payload[3])  << 2U)
                   |  (static_cast<uint32_t>(payload[4])  << 10U))
                   & 0x07FFU;

  rawChannels[3]  =  ((static_cast<uint32_t>(payload[4])  >> 1U)
                   |  (static_cast<uint32_t>(payload[5])  << 7U))
                   & 0x07FFU;

  rawChannels[4]  =  ((static_cast<uint32_t>(payload[5])  >> 4U)
                   |  (static_cast<uint32_t>(payload[6])  << 4U))
                   & 0x07FFU;

  rawChannels[5]  =  ((static_cast<uint32_t>(payload[6])  >> 7U)
                   |  (static_cast<uint32_t>(payload[7])  << 1U)
                   |  (static_cast<uint32_t>(payload[8])  << 9U))
                   & 0x07FFU;

  rawChannels[6]  =  ((static_cast<uint32_t>(payload[8])  >> 2U)
                   |  (static_cast<uint32_t>(payload[9])  << 6U))
                   & 0x07FFU;

  rawChannels[7]  =  ((static_cast<uint32_t>(payload[9])  >> 5U)
                   |  (static_cast<uint32_t>(payload[10]) << 3U))
                   & 0x07FFU;

  // Channels 8-15 (payload bytes 11-21)
  // Note: channel 8 starts at payload[11], not payload[10].  payload[10] carries
  // the upper bits of channel 7 (bits 80-87).  Channel 8 begins at bit 88.
  rawChannels[8]  =  (static_cast<uint32_t>(payload[11])
                   | (static_cast<uint32_t>(payload[12]) << 8U))
                   & 0x07FFU;

  rawChannels[9]  =  ((static_cast<uint32_t>(payload[12]) >> 3U)
                   |  (static_cast<uint32_t>(payload[13]) << 5U))
                   & 0x07FFU;

  rawChannels[10] =  ((static_cast<uint32_t>(payload[13]) >> 6U)
                   |  (static_cast<uint32_t>(payload[14]) << 2U)
                   |  (static_cast<uint32_t>(payload[15]) << 10U))
                   & 0x07FFU;

  rawChannels[11] =  ((static_cast<uint32_t>(payload[15]) >> 1U)
                   |  (static_cast<uint32_t>(payload[16]) << 7U))
                   & 0x07FFU;

  rawChannels[12] =  ((static_cast<uint32_t>(payload[16]) >> 4U)
                   |  (static_cast<uint32_t>(payload[17]) << 4U))
                   & 0x07FFU;

  rawChannels[13] =  ((static_cast<uint32_t>(payload[17]) >> 7U)
                   |  (static_cast<uint32_t>(payload[18]) << 1U)
                   |  (static_cast<uint32_t>(payload[19]) << 9U))
                   & 0x07FFU;

  rawChannels[14] =  ((static_cast<uint32_t>(payload[19]) >> 2U)
                   |  (static_cast<uint32_t>(payload[20]) << 6U))
                   & 0x07FFU;

  rawChannels[15] =  ((static_cast<uint32_t>(payload[20]) >> 5U)
                   |  (static_cast<uint32_t>(payload[21]) << 3U))
                   & 0x07FFU;
}