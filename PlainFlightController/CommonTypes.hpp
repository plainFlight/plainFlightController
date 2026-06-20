#pragma once

#include <cstdint>

/**
 * @enum ModelType
 * @brief Defines all aircraft configurations supported by the flight controller.
 */
enum class ModelType : uint8_t
{
  PLANE_FULL_HOUSE,
  PLANE_FULL_HOUSE_V_TAIL,
  PLANE_ADVANCED_RUDDER_ELEVATOR,
  PLANE_RUDDER_ELEVATOR,
  PLANE_V_TAIL,
  PLANE_FLYING_WING,
  QUAD_X_COPTER,
  QUAD_P_COPTER,
  BI_COPTER,
  CHINOOK_COPTER,
  TRI_COPTER,
  DUAL_COPTER,
  SINGLE_COPTER
};

/**
 * @enum ReceiverType
 * @brief Supported radio protocols.
 */
enum class ReceiverType : uint8_t
{
  SBUS,
  CRSF
};

/**
 * @enum GyroRate
 * @brief Supported gyro rate range.
 */
enum class GyroRate : uint8_t
{
  IS_250_DEGS_SECOND,
  IS_500_DEGS_SECOND
};

/**
 * @enum AircraftDir
 * @brief Used for orientation of the gyro relative to the aircraft.
 *        Note: The ordering is important here, it is used to ensure that 
 *        a valid orientation is selected.
 */
enum class AircraftDir: uint8_t 
{ 
  FRONT, BACK, 
  LEFT, RIGHT, 
  UP, DOWN 
};