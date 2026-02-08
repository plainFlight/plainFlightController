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
* @brief  This class contains methods that all clases may call upon
*/

#include "IMU.hpp"


/**
* @brief    Initialises MPU6050 as used by the IMU class and checks it is communicating.
*/ 
void
IMU::begin()
{
  mpu6050.initialise();

  if (mpu6050.whoAmI() != Mpu6050::MPU6050_ADD)  
  {
    Serial.println("IMU did not intitialise!");
    m_imu.fault = true;
  }
  else
  {
    m_imu.fault = false;
  }
}


/**
* @brief    Indicates if imu faulted.
* @return   true if MPU6050 failed to initialise
*/ 
bool
IMU::isFaulted()
{
  return m_imu.fault;
}


/**
* @brief    Set the madgwick filter weighting.
* @param    Weighting that the madgwick filter should operate at.
* @note     Larger weighting meansfaster reponse but more noise will be present
*/ 
void
IMU::setMadgwickWeighting(float weight)
{
  m_bMadgwick = weight;
}


/**
* @brief  Attitude estimation through sensor fusion - 6DOF
* @param
* @note   See description of Madgwick() for more information. 
* @note   This is a 6DOF implimentation for when magnetometer data is not available (for example when using the recommended MPU6050 IMU for the default setup).
* @note   This is a modified version of the Arduino library:
* @note   https://github.com/arduino-libraries/MadgwickAHRS/tree/master
*/
void
IMU::Madgwick6DOF(const DemandProcessor::FlightState * const flightState) 
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  float gyroX = m_imu.mpu6050.gyro_X * 0.0174533f;
  float gyroY = m_imu.mpu6050.gyro_Y * 0.0174533f;
  float gyroZ = m_imu.mpu6050.gyro_Z * 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * ((-m_q1 * gyroX) - (m_q2 * gyroY) - (m_q3 * gyroZ));
  qDot2 = 0.5f * ((m_q0 * gyroX) + (m_q2 * gyroZ) - (m_q3 * gyroY));
  qDot3 = 0.5f * ((m_q0 * gyroY) - (m_q1 * gyroZ) + (m_q3 * gyroX));
  qDot4 = 0.5f * ((m_q0 * gyroZ) + (m_q1 * gyroY) - (m_q2 * gyroX));

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((m_imu.mpu6050.accel_X == 0.0f) && (m_imu.mpu6050.accel_Y == 0.0f) && (m_imu.mpu6050.accel_Z == 0.0f))) 
  {
    //Normalise accelerometer measurement
    recipNorm = invSqrt((m_imu.mpu6050.accel_X * m_imu.mpu6050.accel_X) + (m_imu.mpu6050.accel_Y * m_imu.mpu6050.accel_Y) + (m_imu.mpu6050.accel_Z * m_imu.mpu6050.accel_Z));
    float accelX = m_imu.mpu6050.accel_X * recipNorm;
    float accelY = m_imu.mpu6050.accel_Y * recipNorm;
    float accelZ = m_imu.mpu6050.accel_Z * recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * m_q0;
    _2q1 = 2.0f * m_q1;
    _2q2 = 2.0f * m_q2;
    _2q3 = 2.0f * m_q3;
    _4q0 = 4.0f * m_q0;
    _4q1 = 4.0f * m_q1;
    _4q2 = 4.0f * m_q2;
    _8q1 = 8.0f * m_q1;
    _8q2 = 8.0f * m_q2;
    q0q0 = m_q0 * m_q0;
    q1q1 = m_q1 * m_q1;
    q2q2 = m_q2 * m_q2;
    q3q3 = m_q3 * m_q3;

    //Gradient decent algorithm corrective step
    s0 = (_4q0 * q2q2) + (_2q2 * accelX) + (_4q0 * q1q1) - (_2q1 * accelY);
    s1 = (_4q1 * q3q3) - (_2q3 * accelX) + (4.0f * q0q0 * m_q1) - (_2q0 * accelY) - _4q1 + (_8q1 * q1q1) + (_8q1 * q2q2) + (_4q1 * accelZ);
    s2 = (4.0f * q0q0 * m_q2) + (_2q0 * accelX) + (_4q2 * q3q3) - (_2q3 * accelY) - _4q2 + (_8q2 * q1q1) + (_8q2 * q2q2) + (_4q2 * accelZ);
    s3 = (4.0f * q1q1 * m_q3) - (_2q1 * accelX) + (4.0f * q2q2 * m_q3) - (_2q2 * accelY);
    recipNorm = invSqrt((s0 * s0) + (s1 * s1) + (s2 * s2) + (s3 * s3)); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= m_bMadgwick * s0;
    qDot2 -= m_bMadgwick * s1;
    qDot3 -= m_bMadgwick * s2;
    qDot4 -= m_bMadgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  m_q0 += qDot1 * m_imu.timeDelta;
  m_q1 += qDot2 * m_imu.timeDelta;
  m_q2 += qDot3 * m_imu.timeDelta;
  m_q3 += qDot4 * m_imu.timeDelta;

  //Normalise quaternion
  recipNorm = invSqrt((m_q0 * m_q0) + (m_q1 * m_q1) + (m_q2 * m_q2) + (m_q3 * m_q3));
  m_q0 *= recipNorm;
  m_q1 *= recipNorm;
  m_q2 *= recipNorm;
  m_q3 *= recipNorm;

  //Compute angles in degrees
  if (DemandProcessor::FlightState::PROP_HANG == *flightState)
  {
    m_imu.pitch = -fastAtan2(((m_q1*m_q3) - (m_q0*m_q2)), 0.5f - ((m_q1*m_q1) + (m_q2*m_q2))) * 57.29577951f;
    if constexpr(Config::REVERSE_PROP_HANG_PITCH_CORRECTIONS)
    {
      m_imu.pitch += 90.0f;
    }
    else
    {
      m_imu.pitch -= 90.0f;
    }
    const float rads = constrain(2.0f * ((m_q0*m_q1) + (m_q2*m_q3)), -0.999999f, 0.999999f);   //Prevent 'not a number' NaN.
    m_imu.yaw = asin(rads) * 57.29577951f;
    if constexpr(Config::REVERSE_PROP_HANG_YAW_CORRECTIONS)
    {
      m_imu.yaw = -m_imu.yaw;
    }
    m_imu.roll = 0.0f;
  }
  else
  {
    m_imu.roll = fastAtan2((m_q0*m_q1) + (m_q2*m_q3), 0.5f - (m_q1*m_q1) - (m_q2*m_q2)) * 57.29577951f;
    const float rads = constrain(-2.0f * ((m_q1*m_q3) - (m_q0*m_q2)), -0.999999f, 0.999999f);   //Prevent 'not a number' NaN.
    m_imu.pitch = asin(rads) * 57.29577951f;

    if (DemandProcessor::FlightState::AP_WIFI == *flightState)
    {
      const float rads = constrain(2.0f * ((m_q0*m_q1) + (m_q2*m_q3)), -0.999999f, 0.999999f);   //Prevent 'not a number' NaN.
      m_imu.yaw = asin(rads) * 57.29577951f;
    }
    else
    {
      m_imu.yaw = 0.0f;
    }
  }

  if constexpr(Config::REVERSE_ROLL_CORRECTIONS)
  {
    m_imu.roll = -m_imu.roll;
  }

  if constexpr(Config::REVERSE_PITCH_CORRECTIONS)
  {
    m_imu.pitch = -m_imu.pitch;
  }

  if constexpr(Config::REVERSE_YAW_CORRECTIONS && Config::USE_PROP_HANG_MODE)
  {
    m_imu.yaw = -m_imu.yaw;
  }

  if constexpr(Config::DEBUG_MADGWICK)
  {
    const uint64_t nowTime = millis();  

    if (m_updateTime <= nowTime)
    {
      Serial.print("Roll: ");
      Serial.print(m_imu.roll);
      Serial.print(", Pitch: ");
      Serial.print(m_imu.pitch);
      if constexpr(Config::USE_PROP_HANG_MODE)
      {
        Serial.print(", yaw: ");
        Serial.print(m_imu.yaw);
      }
      Serial.println();
      m_updateTime = nowTime + 100U;
    }
  }  
}


/**
* @brief    Calibrates gyro to reduce offset and drift, called as part of power oninitialisation.
* @note     Uses sum of variances to detect movement which will upset calibration. 
* @note     If CALIBRATE_MAX_VARIANCE_THRESHOLD is set too low the craft may not calibrate due to IMU noise. 
* @note     CALIBRATE_MAX_VARIANCE_THRESHOLD may need tuning to suit your calibration environment or how noisy your MPU6050 is.
*/
bool
IMU::calibrateGyro()
{
  if constexpr (Config::DEBUG_GYRO_CALIBRATION)
  {
    Serial.println("Calibration...");
  }

  m_calCount++;

  // -----------------------------------------------------------------------------
  // Welford's online algorithm – fixed-point (Q16.16)
  // -----------------------------------------------------------------------------

  int64_t totalM2 = 0;
  int64_t varianceScaled = 0;

  // ---- X axis ----
  const int32_t deltaX = (static_cast<int32_t>(m_imu.mpu6050.rawGyro_X) << Q16_SHIFT) - m_xGyroMean;
  m_xGyroMean += deltaX / static_cast<int32_t>(m_calCount);
  const int32_t delta2X = (static_cast<int32_t>(m_imu.mpu6050.rawGyro_X) << Q16_SHIFT) - m_xGyroMean;
  m_xGyroM2 += (static_cast<int64_t>(deltaX) * static_cast<int64_t>(delta2X)) >> Q16_SHIFT;

  // ---- Y axis ----
  const int32_t deltaY = (static_cast<int32_t>(m_imu.mpu6050.rawGyro_Y) << Q16_SHIFT) - m_yGyroMean;
  m_yGyroMean += deltaY / static_cast<int32_t>(m_calCount);
  const int32_t delta2Y = (static_cast<int32_t>(m_imu.mpu6050.rawGyro_Y) << Q16_SHIFT) - m_yGyroMean;
  m_yGyroM2 += (static_cast<int64_t>(deltaY) * static_cast<int64_t>(delta2Y)) >> Q16_SHIFT;

  // ---- Z axis ----
  const int32_t deltaZ = (static_cast<int32_t>(m_imu.mpu6050.rawGyro_Z) << Q16_SHIFT) - m_zGyroMean;
  m_zGyroMean += deltaZ / static_cast<int32_t>(m_calCount);
  const int32_t delta2Z = (static_cast<int32_t>(m_imu.mpu6050.rawGyro_Z) << Q16_SHIFT) - m_zGyroMean;
  m_zGyroM2 += (static_cast<int64_t>(deltaZ) * static_cast<int64_t>(delta2Z)) >> Q16_SHIFT;

  // ---- Motion Detection Check ----
  if (m_calCount >= CALIBRATE_MIN_SAMPLES_FOR_VARIANCE_CHECK)
  {
    totalM2 = m_xGyroM2 + m_yGyroM2 + m_zGyroM2;
    varianceScaled = totalM2 / static_cast<int64_t>(m_calCount);

    if constexpr (Config::DEBUG_GYRO_CALIBRATION)
    {
      Serial.print("Variance (LSB^2): ");
      Serial.println(varianceScaled >> Q16_SHIFT);
    }

    if (varianceScaled > (static_cast<int64_t>(CALIBRATE_MAX_VARIANCE_THRESHOLD) << Q16_SHIFT))
    {
      if constexpr (Config::DEBUG_GYRO_CALIBRATION)
      {
        Serial.println("Motion detected — restarting calibration");
      }

      m_xGyroMean = 0;
      m_yGyroMean = 0;
      m_zGyroMean = 0;
      m_xGyroM2 = 0;
      m_yGyroM2 = 0;
      m_zGyroM2 = 0;
      m_calCount = 0;

      m_imu.calibrated = false;
      return false;
    }
  }

  // ---- Calibration Completion ----
  if (m_calCount >= CALIBRATE_COUNTS)
  {
    m_imu.calibrated = true;

    m_imu.mpu6050.gyroOffset_X =
      static_cast<int16_t>((m_xGyroMean + Q16_HALF) >> Q16_SHIFT);
    m_imu.mpu6050.gyroOffset_Y =
      static_cast<int16_t>((m_yGyroMean + Q16_HALF) >> Q16_SHIFT);
    m_imu.mpu6050.gyroOffset_Z =
      static_cast<int16_t>((m_zGyroMean + Q16_HALF) >> Q16_SHIFT);

    if constexpr (Config::DEBUG_GYRO_CALIBRATION)
    {
      Serial.println("Calibration complete...");
      Serial.print("x: "); Serial.print(m_imu.mpu6050.gyroOffset_X);
      Serial.print("\ty: "); Serial.print(m_imu.mpu6050.gyroOffset_Y);
      Serial.print("\tz: "); Serial.println(m_imu.mpu6050.gyroOffset_Z);
    }

    return true;
  }

  return false;
}

/**
* @brief    Indicates that the imu has calibrated.
* @return   True when imu is calibrated
*/ 
bool
IMU::calibrated()
{
  return m_imu.calibrated;
}


/**
* @brief    Main method of imu that sequences tasks.
*/ 
void 
IMU::operate(const float tDelta, const DemandProcessor::FlightState * const flightState)
{
  m_imu.timeDelta = tDelta;
  m_i2cReadOk = mpu6050.readData(&m_imu.mpu6050);
  Madgwick6DOF(flightState);
}


/**
* @brief    Gets the current imu data for use by another.
* @return   Structure of imu data.
*/ 
IMU::ImuData* const 
IMU::getImuData() 
{
  return &m_imu;
}


