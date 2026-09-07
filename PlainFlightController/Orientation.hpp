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
* @file   Orientation.hpp
* @brief  Device-agnostic IMU orientation/axis-remap code, shared by every sensor driver.
*/
#pragma once

#include <cstdint>
#include <array>
#include "Config.hpp"

/**
* @brief    Converts an AircraftDir enum value to a unit vector in {front, left, up} coordinate space.
* @param    d  The aircraft direction to convert.
* @return   A 3-element array representing the unit vector for the given direction.
*/
constexpr std::array<int16_t, 3> dirToVec(AircraftDir d) 
{
    if (d == AircraftDir::FRONT) return {{  1,  0,  0 }};
    if (d == AircraftDir::BACK)  return {{ -1,  0,  0 }};
    if (d == AircraftDir::LEFT)  return {{  0,  1,  0 }};
    if (d == AircraftDir::RIGHT) return {{  0, -1,  0 }};
    if (d == AircraftDir::UP)    return {{  0,  0,  1 }};
    if (d == AircraftDir::DOWN)  return {{  0,  0, -1 }};
    return {{ 0, 0, 0 }};
}

/**
* @brief    Builds a 3x3 rotation matrix at compile time to transform IMU-space
*           acceleration and rotation vectors into aircraft coordinate space.
*           The Z axis is derived as the cross product of the configured IMU X and Y axes.
* @return   A 3x3 matrix where each row maps an aircraft axis to the corresponding IMU axis.
*/
namespace Orientation 
{
  using Matrix3x3 = std::array<std::array<int16_t, 3>, 3>;

  // Build the rotation matrix required to transform the acceleration and rotation
  // vectors back into the aircraft coordinate space
  constexpr Matrix3x3 getMatrix() 
  {
    // x and y are given from the configuration
    std::array<int16_t, 3> imuX = dirToVec(Config::IMU_PLUS_X);
    std::array<int16_t, 3> imuY = dirToVec(Config::IMU_PLUS_Y);

    // Z is orthogonal to x and y, we can derive it using the cross product
    // of IMU X and IMU Y. Note the sign change for the second term.
    std::array<int16_t, 3> imuZ = 
    {
        static_cast<int16_t>(imuX[1]*imuY[2] - imuX[2]*imuY[1]),
        static_cast<int16_t>(imuX[2]*imuY[0] - imuX[0]*imuY[2]),  // negative
        static_cast<int16_t>(imuX[0]*imuY[1] - imuX[1]*imuY[0])
    };

    // --- Now fill all 9 elements ---
    // Row index = aircraft axis {FRONT/BACK=0, LEFT/RIGHT=1, UP/DOWN=2}
    // Column index = IMU axis {X=0, Y=1, Z=2}
    return Matrix3x3
    {{
        {{imuX[0], imuY[0], imuZ[0]}},  // aircraft X (FRONT/BACK)
        {{imuX[1], imuY[1], imuZ[1]}},  // aircraft Y (LEFT/RIGHT)
        {{imuX[2], imuY[2], imuZ[2]}}   // aircraft Z (UP/DOWN)
    }};
  }

  /**
  * @brief    Precomputed rotation matrix derived from Config::IMU_PLUS_X and Config::IMU_PLUS_Y.
  *           Used at runtime to remap raw IMU data into aircraft coordinate space.
  */
  inline constexpr Matrix3x3 final_matrix = getMatrix();

  // We assume that the orientation matrix only contains -1, 0, 1.  Enforce this.
  static_assert(
    []{
      for (const auto& row : final_matrix)
      {
        for (const auto& v : row)
        {
          if ((v < -1) || (v > 1)) { return false; }
        }
      }
      return true;
    }(),
    "Orientation matrix must only ever contain unit coefficients (-1, 0, 1)"
  );

  // This relies on the AircraftDir enum having sequential opposites to check for sharing opposited direction
  static_assert(
    Config::IMU_PLUS_X != Config::IMU_PLUS_Y &&
    Config::IMU_PLUS_X != (static_cast<AircraftDir>(static_cast<int>(Config::IMU_PLUS_Y) ^ 1)),
    "Invalid IMU Configuration: IMU_PLUS_X and IMU_PLUS_Y cannot share the same physical dimension!"
  );
};

/**
* @brief    Multiplies a single row of the orientation matrix against the raw IMU axis values,
*           remapping one aircraft axis from IMU space. Since the matrix contains only unit vectors
*           (0, 1, or -1), only one column per row will be non-zero.
* @tparam   Row  The aircraft axis row index (0=front/back, 1=left/right, 2=up/down).
* @param    x    Raw IMU X axis value.
* @param    y    Raw IMU Y axis value.
* @param    z    Raw IMU Z axis value.
* @note     Changing sign of values at full saturation (max value) can cause unwanted behavior
* @note     Additional care is take here to ensure we don't try have errors when changing sign
* @return   The remapped axis value in aircraft coordinate space.
*/
template<size_t Row>
inline int16_t remapAxis(int16_t x, int16_t y, int16_t z) 
{
  int32_t result;

  if constexpr (Orientation::final_matrix[Row][0] != 0)
  {
    result = static_cast<int32_t>(x) * static_cast<int32_t>(Orientation::final_matrix[Row][0]);
  }
  else if constexpr (Orientation::final_matrix[Row][1] != 0)
  {
    result = static_cast<int32_t>(y) * static_cast<int32_t>(Orientation::final_matrix[Row][1]);
  }
  else
  {
    result = static_cast<int32_t>(z) * static_cast<int32_t>(Orientation::final_matrix[Row][2]);
  }

  //Saturate explicitly rather than allowing an implicit narrowing conversion to wrap.
  if (result > static_cast<int32_t>(INT16_MAX))
  { 
    result = static_cast<int32_t>(INT16_MAX); 
  }
  else if (result < static_cast<int32_t>(INT16_MIN))
  { 
    result = static_cast<int32_t>(INT16_MIN); 
  }
  else
  { // Nothing here
  }

  return static_cast<int16_t>(result);
}
