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
#include "InternalConfig.hpp"
#include "Settings.hpp"


class Configurator
{
  public:
    Configurator();
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
    static constexpr char KEY_SCHEMA[] = "Schema";

    void applyDefault(const Settings::Row& row);
    bool readSetting(const Settings::Row& row, JsonVariantConst fileValue);

    //Rates are limited to below the gyro range, see InternalConfig::MAX_RATE_DEGS_SECOND
    static constexpr float MAX_RATE = static_cast<float>(InternalConfig::MAX_RATE_DEGS_SECOND);
    static constexpr float MIN_RATE = 10.0f;
    static constexpr float rateDefault(const float rate) {return (rate < MAX_RATE) ? rate : MAX_RATE;};

    FileSystem::NonVolatileData m_fileData = {};

    /**
    * @brief  Every setting in the settings file. Defaults are applied from here at start-up.
    * @note   Every default must be inside its own limits and safe to fly at every gyro range, because a value
    *         outside the limits in the file loads the default at boot. Every value between min and max must
    *         also be safe to fly, because the web page accepts any of them. A max angle of 0 divides by zero,
    *         and a rate of 0 gives no stick authority, so neither is allowed.
    * @note   A change to a key or a scale needs a new Settings::SCHEMA.
    */
    static constexpr uint32_t SETTINGS_COUNT = 25U;   //Must equal the number of rows, otherwise compiling fails
    const Settings::Row m_settings[SETTINGS_COUNT] = {
      //key            storage                          default             min       max      scale
      {"Pitch P",      &m_fileData.gains.pitch.p,       25.0f,             0.0f,      200.0f,     1},
      {"Pitch I",      &m_fileData.gains.pitch.i,       50.0f,             0.0f,      500.0f,     1},
      {"Pitch D",      &m_fileData.gains.pitch.d,       0.0f,              0.0f,      200.0f,     10},
      {"Pitch F",      &m_fileData.gains.pitch.ff,      18.0f,             0.0f,      50.0f,      1},
      {"Roll P",       &m_fileData.gains.roll.p,        25.0f,             0.0f,      200.0f,     1},
      {"Roll I",       &m_fileData.gains.roll.i,        50.0f,             0.0f,      500.0f,     1},
      {"Roll D",       &m_fileData.gains.roll.d,        0.0f,              0.0f,      200.0f,     10},
      {"Roll F",       &m_fileData.gains.roll.ff,       18.0f,             0.0f,      50.0f,      1},
      {"Yaw P",        &m_fileData.gains.yaw.p,         25.0f,             0.0f,      200.0f,     1},
      {"Yaw I",        &m_fileData.gains.yaw.i,         50.0f,             0.0f,      500.0f,     1},
      {"Yaw D",        &m_fileData.gains.yaw.d,         0.0f,              0.0f,      200.0f,     10},
      {"Yaw F",        &m_fileData.gains.yaw.ff,        10.0f,             0.0f,      50.0f,      1},
      {"Pitch Rate",   &m_fileData.rates.pitch,         rateDefault(200),  MIN_RATE,  MAX_RATE,   100},
      {"Roll Rate",    &m_fileData.rates.roll,          rateDefault(200),  MIN_RATE,  MAX_RATE,   100},
      {"Yaw Rate",     &m_fileData.rates.yaw,           rateDefault(150),  MIN_RATE,  MAX_RATE,   100},
      {"Pitch Angle",  &m_fileData.maxAngle.pitch,      55.0f,             1.0f,      90.0f,      100},
      {"Roll Angle",   &m_fileData.maxAngle.roll,       55.0f,             1.0f,      90.0f,      100},
      {"Pitch Trim",   &m_fileData.levelTrim.pitch,     0.0f,              -25.0f,    25.0f,      1},
      {"Roll Trim",    &m_fileData.levelTrim.roll,      0.0f,              -25.0f,    25.0f,      1},
      {"Yaw Trim",     &m_fileData.levelTrim.yaw,       0.0f,              -25.0f,    25.0f,      1},
      {"Servo1 Trim",  &m_fileData.servoTrim.servo1,    0.0f,              -200.0f,   200.0f,     1},
      {"Servo2 Trim",  &m_fileData.servoTrim.servo2,    0.0f,              -200.0f,   200.0f,     1},
      {"Servo3 Trim",  &m_fileData.servoTrim.servo3,    0.0f,              -200.0f,   200.0f,     1},
      {"Servo4 Trim",  &m_fileData.servoTrim.servo4,    0.0f,              -200.0f,   200.0f,     1},
      {"Batt Scale",   &m_fileData.batteryScaler,       0.00357f,          0.0f,      0.05f,      1},
    };

    float m_batteryVoltage = 0.0f;
    float m_pitch          = 0.0f;
    float m_roll           = 0.0f;
    float m_yaw            = 0.0f;

    //Objects
    WifiConfig wifi = WifiConfig(&m_fileData, &m_batteryVoltage, &m_pitch, &m_roll, &m_yaw);
    FileSystem fileSys;
};