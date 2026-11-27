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
* @file   Settings.hpp
* @brief  Describes one setting held in the settings file, and the unit conversion and range checks applied to it.
*/
#pragma once

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstddef>

namespace Settings
{
  /**
  * @brief  Version of the settings file layout, written to the file as "Schema".
  * @note   Increase it only when an existing key is renamed or removed, or when its units or scale change.
  *         Adding a key does not need a new version.
  *         A file with no "Schema" key is schema 0, which has the same keys and units as schema 1.
  */
  static constexpr uint32_t SCHEMA = 1U;

  /**
  * @brief  Where a setting is stored: a whole number or a decimal. Exactly one of the pointers is set.
  * @note   There is no default constructor, so a table row left out of an initialiser fails to compile.
  */
  struct Storage
  {
    /**
    * @brief  Stores the setting as a whole number.
    * @param  value  Where the whole number is held.
    */
    constexpr Storage(int32_t* const value) : intValue(value), floatValue(nullptr) {}

    /**
    * @brief  Stores the setting as a decimal.
    * @param  value  Where the decimal is held.
    */
    constexpr Storage(float* const value) : intValue(nullptr), floatValue(value) {}

    int32_t* const intValue;
    float* const floatValue;
  };

  /**
  * @brief  One setting. Default and limits are in the units shown on the web page.
  * @note   Stored value = shown value x scale.
  */
  struct Row
  {
    const char* key;      //Key in the settings file
    Storage storage;
    float defaultValue;
    float min;
    float max;
    int32_t scale;
  };

  /**
  * @brief  Converts a shown value to a stored whole number, rounded to the nearest.
  * @note   Without rounding 0.29 x 100 would truncate to 28.
  * @param  shown  Value in the units shown on the web page.
  * @param  scale  Multiplier from shown units to stored units.
  * @return Stored value, shown x scale rounded to the nearest whole number.
  */
  inline int32_t toStoredInt(const float shown, const int32_t scale)
  {
    return static_cast<int32_t>(std::lround(shown * static_cast<float>(scale)));
  }

  /**
  * @brief  Converts a shown value to a stored decimal.
  * @param  shown  Value in the units shown on the web page.
  * @param  scale  Multiplier from shown units to stored units.
  * @return Stored value, shown x scale.
  */
  inline float toStoredFloat(const float shown, const int32_t scale)
  {
    return shown * static_cast<float>(scale);
  }

  /**
  * @brief  True when min <= value <= max.
  * @param  value  Value to check.
  * @param  min    Lowest allowed value.
  * @param  max    Highest allowed value.
  * @return true if value is within min and max inclusive, false otherwise.
  */
  inline bool isInRange(const int32_t value, const int32_t min, const int32_t max)
  {
    return (value >= min) && (value <= max);
  }

  /**
  * @brief  True when min <= value <= max.
  * @note   False for NaN, as every comparison with NaN is false.
  * @param  value  Value to check.
  * @param  min    Lowest allowed value.
  * @param  max    Highest allowed value.
  * @return true if value is within min and max inclusive, false otherwise.
  */
  inline bool isInRange(const float value, const float min, const float max)
  {
    return (value >= min) && (value <= max);
  }

  /**
  * @brief  Writes the board's unique name, the short board name and the last 3 bytes of the factory MAC, e.g. "XIAO-3F2A1C".
  * @param  name        Buffer for the name.
  * @param  size        Size of the buffer.
  * @param  shortName   Short board name.
  * @param  factoryMac  Factory MAC as returned by ESP.getEfuseMac(). Its first byte is the low-order byte.
  * @note   The first 3 bytes of a MAC identify the vendor, so only the last 3 differ between boards.
  */
  inline void makeBoardName(char* const name, const size_t size, const char* const shortName, const uint64_t factoryMac)
  {
    const unsigned int mac3 = static_cast<unsigned int>((factoryMac >> 24) & 0xFFU);
    const unsigned int mac4 = static_cast<unsigned int>((factoryMac >> 32) & 0xFFU);
    const unsigned int mac5 = static_cast<unsigned int>((factoryMac >> 40) & 0xFFU);

    snprintf(name, size, "%s-%02X%02X%02X", shortName, mac3, mac4, mac5);
  }
}
