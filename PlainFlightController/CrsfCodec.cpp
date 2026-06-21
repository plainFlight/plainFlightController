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
* MISRA alignment  (Is this over the top?)
*   Explicit static_cast<> is used throughout in preference to relying on
*   implicit arithmetic promotions.  In particular:
*     - uint8_t operands are cast to uint32_t before shift operations to
*       avoid the implicit promotion to signed int that the C++ standard
*       applies to sub-int types.
*     - Results of arithmetic on wider types are explicitly narrowed back to
*       uint8_t when stored into byte-array slots or uint8_t variables.
*
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
CrsfCodec::crc8DvbS2(const uint8_t crc, const uint8_t a)
{
  // Use a local variable for the calculation to keep input parameters immutable.
  uint8_t crcResult = static_cast<uint8_t>(static_cast<uint32_t>(crc) ^ static_cast<uint32_t>(a));

  for (uint32_t i = 0U; i < 8U; i++)
  {
    if ((crcResult & 0x80U) != 0U)
    {
      // Cast to uint32_t before shifting to avoid signed-int promotion of uint8_t.
      crcResult = static_cast<uint8_t>((static_cast<uint32_t>(crcResult) << 1U) ^ 0xD5U);
    }
    else
    {
      crcResult = static_cast<uint8_t>(static_cast<uint32_t>(crcResult) << 1U);
    }
  }
  return crcResult;
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
CrsfCodec::calculateCrc(const uint8_t* const data, const uint8_t length)
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
*            result = ((p[2]>>6) | (p[3]<<2) | (p[4]<<10)) & CHANNEL_VALUE_MASK
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
CrsfCodec::unpackChannels(const uint8_t* const payload, uint32_t (&rawChannels)[NUM_CHANNELS])
{
  // Channels 0-7 (payload bytes 0-10)
  rawChannels[0]  =  (static_cast<uint32_t>(payload[0])
                   | (static_cast<uint32_t>(payload[1])  << 8U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[1]  =  ((static_cast<uint32_t>(payload[1])  >> 3U)
                   |  (static_cast<uint32_t>(payload[2])  << 5U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[2]  =  ((static_cast<uint32_t>(payload[2])  >> 6U)
                   |  (static_cast<uint32_t>(payload[3])  << 2U)
                   |  (static_cast<uint32_t>(payload[4])  << 10U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[3]  =  ((static_cast<uint32_t>(payload[4])  >> 1U)
                   |  (static_cast<uint32_t>(payload[5])  << 7U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[4]  =  ((static_cast<uint32_t>(payload[5])  >> 4U)
                   |  (static_cast<uint32_t>(payload[6])  << 4U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[5]  =  ((static_cast<uint32_t>(payload[6])  >> 7U)
                   |  (static_cast<uint32_t>(payload[7])  << 1U)
                   |  (static_cast<uint32_t>(payload[8])  << 9U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[6]  =  ((static_cast<uint32_t>(payload[8])  >> 2U)
                   |  (static_cast<uint32_t>(payload[9])  << 6U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[7]  =  ((static_cast<uint32_t>(payload[9])  >> 5U)
                   |  (static_cast<uint32_t>(payload[10]) << 3U))
                   & CHANNEL_VALUE_MASK;

  // Channels 8-15 (payload bytes 11-21)
  // Note: channel 8 starts at payload[11], not payload[10].  payload[10] carries
  // the upper bits of channel 7 (bits 80-87).  Channel 8 begins at bit 88.
  rawChannels[8]  =  (static_cast<uint32_t>(payload[11])
                   | (static_cast<uint32_t>(payload[12]) << 8U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[9]  =  ((static_cast<uint32_t>(payload[12]) >> 3U)
                   |  (static_cast<uint32_t>(payload[13]) << 5U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[10] =  ((static_cast<uint32_t>(payload[13]) >> 6U)
                   |  (static_cast<uint32_t>(payload[14]) << 2U)
                   |  (static_cast<uint32_t>(payload[15]) << 10U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[11] =  ((static_cast<uint32_t>(payload[15]) >> 1U)
                   |  (static_cast<uint32_t>(payload[16]) << 7U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[12] =  ((static_cast<uint32_t>(payload[16]) >> 4U)
                   |  (static_cast<uint32_t>(payload[17]) << 4U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[13] =  ((static_cast<uint32_t>(payload[17]) >> 7U)
                   |  (static_cast<uint32_t>(payload[18]) << 1U)
                   |  (static_cast<uint32_t>(payload[19]) << 9U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[14] =  ((static_cast<uint32_t>(payload[19]) >> 2U)
                   |  (static_cast<uint32_t>(payload[20]) << 6U))
                   & CHANNEL_VALUE_MASK;

  rawChannels[15] =  ((static_cast<uint32_t>(payload[20]) >> 5U)
                   |  (static_cast<uint32_t>(payload[21]) << 3U))
                   & CHANNEL_VALUE_MASK;
}

/**
* @brief   Build a complete CRSF Battery Sensor frame (type 0x08) into buf.
* @details The CRSF battery frame carries four fields in big-endian byte order:
*
*            Field           Wire bytes   Wire unit          Source
*            --------------- ------------ ------------------ -----------------------
*            Voltage         2            100 mV per LSB     voltageVolts × 10
*            Current         2            10 mA per LSB      zero (sensor absent)
*            Capacity used   3            mAh                zero (sensor absent)
*            Remaining       1            percent (0–100)    zero (sensor absent)
*
*          The unit conversion from volts to centivolts (× 100) is done here.
*          When a current sensor is added, its raw value in amps would
*          be converted to centiamps (× 100) at the corresponding field.
*
*          Frame layout:
*            [0]     Sync byte  (ADDR_FLIGHT_CONTROLLER = 0xC8)
*            [1]     Frame length = BATTERY_FRAME_SIZE - 2 = 10
*            [2]     Frame type  (FRAMETYPE_BATTERY = 0x08)  <-- CRC region start
*            [3–4]   Voltage, big-endian uint16_t, centivolts
*            [5–6]   Current, big-endian uint16_t, zero
*            [7–9]   Capacity used, big-endian 24-bit, zero
*            [10]    Remaining percent, zero
*            [11]    CRC8 DVB-S2 over bytes [2..10]          <- CRC region end
*
*          Battery voltage is always positive; the cast from float to uint16_t is
*          safe for any realistic pack voltage (a 65V pack would be the limit,
*          far beyond practical use).
*
* @param   buf           Output buffer; must be at least MAX_FRAME_SIZE bytes.
* @param   voltageVolts  Battery pack voltage in volts (e.g. 12.6f).
* @return  Number of bytes written into buf (always BATTERY_FRAME_SIZE = 12).
*/
uint8_t
CrsfCodec::buildBatteryFrame(uint8_t* buf, const float voltageVolts)
{
  uint8_t index = 0U;
 
  // Sync byte: identifies this frame as originating from the flight controller.
  buf[index++] = ADDR_FLIGHT_CONTROLLER;
 
  // Frame length field: counts type(1) + payload(8) + CRC(1) = 10.
  // Excludes the sync byte and the length byte itself.
  buf[index++] = static_cast<uint8_t>(BATTERY_FRAME_SIZE - 2U);
 
  // Mark where the CRC region begins: the CRC covers from the type byte
  // through to the last payload byte.
  const uint8_t crcStart = index;
 
  // Frame type.
  buf[index++] = FRAMETYPE_BATTERY;
 
  // Voltage: convert from volts to decivolts (100 mV per LSB) Note: this is not correct
  // according to the TBS CRSF Spec.  However the Spec suggests LSB = 10 uV which makes
  // no sense for a signed 16 bit value.  The value here results in the correct reading
  // at the transmitter.
  // Data type is int16_t per spec; static_cast to uint32_t handles the bit packing safely.
  const int16_t voltageDv = static_cast<int16_t>(voltageVolts * 10.0f);
  buf[index++] = static_cast<uint8_t>((static_cast<uint32_t>(voltageDv) >> 8U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>( static_cast<uint32_t>(voltageDv)        & 0xFFU);
 
  // Current: sensor not yet fitted; transmitted as zero.
  buf[index++] = 0U;
  buf[index++] = 0U;
 
  // Capacity used (24-bit field): not yet derived; transmitted as zero.
  buf[index++] = 0U;
  buf[index++] = 0U;
  buf[index++] = 0U;
 
  // Remaining percentage: not yet derived; transmitted as zero.
  buf[index++] = 0U;
 
  // CRC8 DVB-S2: calculated over type + payload bytes.
  buf[index] = calculateCrc(&buf[crcStart], static_cast<uint8_t>(index - crcStart));
  index++;
 
  // index is now BATTERY_FRAME_SIZE (12).
  return index;
}
 
/**
* @brief   Build a complete CRSF GPS Sensor frame (type 0x02) into buf.
* @details The CRSF GPS frame carries six fields in big-endian byte order:
*
*            Field           Wire bytes   Wire unit          Source
*            --------------- ------------ ------------------ -----------------------
*            latitude        4            degree * 10`000`000
*            longitude       4            degree * 10`000`000
*            groundspeed     2            km/h * 100
*            heading         2            degree * 100
*            altitude        2            meter + 1000m offset
*            satellites      1            # of sats in view
*
*          Frame layout:
*            [0]     Sync byte  (ADDR_FLIGHT_CONTROLLER = 0xC8)
*            [1]     Frame length = BATTERY_FRAME_SIZE - 2 = 10
*            [2]     Frame type  (FRAMETYPE_BATTERY = 0x08)  <-- CRC region start
*            [3–6]   Latitude, big-endian uint32_t, degrees * 10E7
*            [7–10]  Longitude, big-endian uint32_t, degrees * 10E7
*            [11–12] Ground Speed, big-endian 16-bit, km/h * 100
*            [13-14] Heading, big-endian 16-bit, degrees * 100
*            [15-16] Altitude, big-endian 16-bit, metres + 1000
*            [17]    #Satellites, big-endian 8-bit, N
*            [18]    CRC8 DVB-S2 over bytes [2..17]          <- CRC region end
*
*
* @param   buf   Output buffer; must be at least MAX_FRAME_SIZE bytes.
* @param   data  GnssData structure
* @return  Number of bytes written into buf (always GPS_FRAME_SIZE = 19).
*/

uint8_t
CrsfCodec::buildGpsFrame(uint8_t* buf, const GnssData& data)
{
  uint8_t index = 0U;
 
  // Sync byte: identifies this frame as originating from the flight controller.
  buf[index++] = ADDR_FLIGHT_CONTROLLER;
 
  // Frame length field: counts type(1) + payload(8) + CRC(1) = 10.
  // Excludes the sync byte and the length byte itself.
  buf[index++] = static_cast<uint8_t>(GPS_FRAME_SIZE - 2U);
 
  // Mark where the CRC region begins: the CRC covers from the type byte
  // through to the last payload byte.
  const uint8_t crcStart = index;
 
  // Frame type.
  buf[index++] = FRAMETYPE_GPS;
  const uint32_t latBits = static_cast<uint32_t>(data.latitude);  // cast to avoid right shift signed int
  buf[index++] = static_cast<uint8_t>((latBits >> 24U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>((latBits >> 16U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>((latBits >> 8U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>(latBits & 0xFFU);
  const uint32_t longBits = static_cast<uint32_t>(data.longitude);  // cast to avoid right shift signed int
  buf[index++] = static_cast<uint8_t>((longBits >> 24U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>((longBits >> 16U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>((longBits >> 8U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>(longBits & 0xFFU);
  const uint16_t gSpeedDV = static_cast<uint16_t>(data.gSpeed * 0.36f); // mm/s to kph * 100
  buf[index++] = static_cast<uint8_t>((gSpeedDV >> 8U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>(gSpeedDV & 0xFFU);
  const uint16_t headMotDv = static_cast<uint16_t>(data.headMot / 1000L);  // degrees * 1E5 to degrees * 100
  buf[index++] = static_cast<uint8_t>((headMotDv >> 8U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>(headMotDv & 0xFFU);
  const int32_t altMetres = data.altMSL / 1000L;  // mm to m
  const int32_t altWithOffset = altMetres + 1000;
  const uint16_t altMSLDv = (altWithOffset > 0) ?
                              static_cast<uint16_t>(altWithOffset): 0U;  // Guard against (unlikely) - ve to unsigned
  buf[index++] = static_cast<uint8_t>((altMSLDv >> 8U) & 0xFFU);
  buf[index++] = static_cast<uint8_t>(altMSLDv & 0xFFU);
  buf[index++] = data.satellites;
 
   // CRC8 DVB-S2: calculated over type + payload bytes.
  buf[index] = calculateCrc(&buf[crcStart], static_cast<uint8_t>(index - crcStart));
  index++;
 
  // index is now GPS_FRAME_SIZE (19).
  return index;
}
