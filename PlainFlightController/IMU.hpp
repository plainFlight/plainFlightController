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
* @file   IMU.hpp
* @brief  This class contains timer methods to handle IMU tasks.
*/
#pragma once

#include <inttypes.h>
#include <Arduino.h>
#include "Utilities.hpp"
#include "Mpu6050.hpp"
#include "Config.hpp"
#include "DemandProcessor.hpp"



/**
* @class IMU
*/

class IMU : public Utilities
{
  public:
    struct ImuData
    {
      Mpu6050::MpuData mpu6050;
      float roll;
      float pitch;
      float yaw;
      float timeDelta;  
      bool calibrated;
      bool fault;
    };

    static constexpr float MADGWICK_WARM_UP_WEIGHTING  = 5.0f;
    static constexpr float MADGWICK_FLIGHT_WEIGHTING   = 0.01f;

    IMU(){};  
    void begin();
    void operate(const float tdelta, const DemandProcessor::FlightState * const flightState);
    void Madgwick6DOF(const DemandProcessor::FlightState * const flightState);
    bool calibrateGyro();
    void setMadgwickWeighting(float weight);
    bool calibrated();
    bool isFaulted();
    ImuData* const getImuData();
    bool isOk() const {return m_i2cReadOk;};

  private:
    /**@brief Gyro calibration constants...
    * @note   CALIBRATE_MAX_VARIANCE_THRESHOLD - A threshold above which calibration will fail due to craft movement. 
    * @note   ...Craft must be still to pass calibration, only increase CALIBRATE_MAX_VARIANCE_THRESHOLD if craft cannot be held still enough.
    * @note   ...But understand increasing threshold may reduce flight controller performance due to additonal gyro offsets/noise.
    */
    static constexpr uint32_t CALIBRATE_MAX_VARIANCE_THRESHOLD          = 100U; // Sum of variances used to pass/fail gyro calibration. Increase to desensitise.
    static constexpr uint32_t CALIBRATE_MIN_SAMPLES_FOR_VARIANCE_CHECK  = 100U; // After n samples, check variance
    static constexpr uint32_t CALIBRATE_COUNTS      = 1000U; 
    //Q16.16 constants for Welford's algorithm used for calibration
    static constexpr int32_t Q16_SHIFT = 16;
    static constexpr int32_t Q16_SCALE = 1 << Q16_SHIFT;     // 65536 when Q16_SHIFT is 16
    static constexpr int32_t Q16_HALF  = 1 << (Q16_SHIFT-1); // 32768 when Q16_SHIFT is 16

    //Variables
    float m_bMadgwick = 0.0f;
    ImuData m_imu = {0};
    //Calibration variables
    int32_t  m_xGyroMean;   // actually mean × 65536  (Q16.16 format, but signed)
    int32_t  m_yGyroMean;
    int32_t  m_zGyroMean;

    int64_t  m_xGyroM2;     // scaled sum of squared deviations × 65536
    int64_t  m_yGyroM2;
    int64_t  m_zGyroM2;
    uint32_t m_calCount = 0U;
    bool m_i2cReadOk = true;
    uint64_t m_updateTime = 0U;

    float m_q0 = 1.0f; //Initialize quaternion for madgwick filter 
    float m_q1 = 0.0f;
    float m_q2 = 0.0f;
    float m_q3 = 0.0f;

    //Objects
    Mpu6050 mpu6050;
};