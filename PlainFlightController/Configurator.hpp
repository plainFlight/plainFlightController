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
* @file   Configurator.hpp
* @brief  This class contains methods that controls the configurator, including FileSystem and WifiConfig.
*/

#pragma once

#include <ArduinoJson.h>
#include "WifiConfig.hpp"
#include "FileSystem.hpp"
#include "PIDF.hpp"
#include "Config.hpp"


class Configurator
{
  public:
    Configurator(){};
    ~Configurator(){};
    bool begin();
    void operate();
    bool readConfig();
    bool writeConfig();

    PIDF::Gains const * const getPitchGains() const {return &m_fileData.gains.pitch;};
    PIDF::Gains const * const getRollGains() const {return &m_fileData.gains.roll;};
    PIDF::Gains const * const getYawGains() const {return &m_fileData.gains.yaw;};
    FileSystem::Rates const * const getRates() const {return &m_fileData.rates;};
    FileSystem::MaxAngle const * const getMaxAngles() const {return &m_fileData.maxAngle;};
    const int32_t getPitchRate() const {return m_fileData.rates.pitch;};
    const int32_t getRollRate() const {return m_fileData.rates.roll;};
    const int32_t getYawRate() const {return m_fileData.rates.yaw;};
    const int32_t getMaxPitchAngle() const {return m_fileData.maxAngle.pitch;};
    const int32_t getMaxRollAngle() const {return m_fileData.maxAngle.roll;};
    const float getPitchTrim() const {return m_fileData.levelTrim.pitch;};
    const float getRollTrim() const {return m_fileData.levelTrim.roll;};
    const float getYawTrim() const {return m_fileData.levelTrim.yaw;};
    FileSystem::ServoTrims const * const getServoTrims() const {return &m_fileData.servoTrim;};
    const float getBatteryScaler() const {return m_fileData.batteryScaler;};

    void updateBatteryVoltage(const float batteryVoltage) {m_batteryVoltage = batteryVoltage;};
    void updateImuAngles(const float pitch, const float roll, const float yaw) {m_pitch = pitch; m_roll = roll; m_yaw = yaw;};

  private:
    //Default values for file system
    //Rates
    static constexpr  int32_t MAX_PITCH_RATE_DEGS_x100  = 20000;
    static constexpr  int32_t MAX_ROLL_RATE_DEGS_x100   = 20000;
    static constexpr  int32_t MAX_YAW_RATE_DEGS_x100    = 15000;
    //Max angles allowed when in levelled mode (angles * 100)
    static constexpr  int32_t MAX_PITCH_ANGLE_DEGS_x100 = 5500;
    static constexpr  int32_t MAX_ROLL_ANGLE_DEGS_x100 = 5500;
    //Trims
    static constexpr  float LEVELLED_ROLL_TRIM        = 0.0f;
    static constexpr  float LEVELLED_PITCH_TRIM       = 0.0f;
    static constexpr  int32_t SERVO_TRIM              = 0;
    //Gains
    static constexpr  int32_t PITCH_P_GAIN              = 25;
    static constexpr  int32_t PITCH_I_GAIN              = 50;
    static constexpr  int32_t PITCH_D_GAIN              = 0;
    static constexpr  int32_t PITCH_F_GAIN              = 18;
    static constexpr  int32_t ROLL_P_GAIN               = 25;
    static constexpr  int32_t ROLL_I_GAIN               = 50;
    static constexpr  int32_t ROLL_D_GAIN               = 0;
    static constexpr  int32_t ROLL_F_GAIN               = 18;
    static constexpr  int32_t YAW_P_GAIN                = 25;
    static constexpr  int32_t YAW_I_GAIN                = 50;
    static constexpr  int32_t YAW_D_GAIN                = 0;
    static constexpr  int32_t YAW_F_GAIN                = 10;
    //Battery scaler
    static constexpr float BATTERY_SCALER               = 0.00357f;

    //Variables
    FileSystem::NonVolatileData m_fileData = {
      {{PITCH_P_GAIN, PITCH_I_GAIN, PITCH_D_GAIN, PITCH_F_GAIN},
      {ROLL_P_GAIN, ROLL_I_GAIN, ROLL_D_GAIN, ROLL_F_GAIN},
      {YAW_P_GAIN, YAW_I_GAIN, YAW_D_GAIN, YAW_F_GAIN}},
      {MAX_PITCH_RATE_DEGS_x100, MAX_ROLL_RATE_DEGS_x100, MAX_YAW_RATE_DEGS_x100}, 
      {MAX_PITCH_ANGLE_DEGS_x100, MAX_ROLL_ANGLE_DEGS_x100},
      {LEVELLED_PITCH_TRIM, LEVELLED_ROLL_TRIM},
      {SERVO_TRIM, SERVO_TRIM, SERVO_TRIM, SERVO_TRIM},
      BATTERY_SCALER,          
      };

    float m_batteryVoltage = 0.0f;
    float m_pitch          = 0.0f;
    float m_roll           = 0.0f;
    float m_yaw            = 0.0f;

    //Objects
    WifiConfig wifi = WifiConfig(&m_fileData, &m_batteryVoltage, &m_pitch, &m_roll, &m_yaw);
    FileSystem fileSys;
};