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
* @file   FlightControl.hpp
* @brief  This class contains methods to operate the flight controller.
*/
#pragma once

#include <inttypes.h>
#include <Arduino.h>
#include "Utilities.hpp"
#include "DemandProcessor.hpp"
#include "BatteryMonitor.hpp"
#include "LED.hpp"
#include "PIDF.hpp"
#include "IMU.hpp"
#include "Config.hpp"
#include "RxBase.hpp"
#include "GnssDriver.hpp"
#include "TelemetryManager.hpp"
#include "Configurator.hpp"
#include "FileSystem.hpp"
#include "ModelTypes.hpp"


/**
* @class FlightControl
*/

class FlightControl : public Utilities
{
  public:  
    FlightControl(){};
    void operate();
    void begin();

    /**
    * @brief  Current flight state as decided by the demand processor.
    * @return The flight state for this loop.
    */
    DemandProcessor::FlightState getFlightState() const {return m_flightState;};

    /**
    * @brief  Latest IMU data, including attitude from the Madgwick filter.
    * @return Pointer to the IMU data structure.
    */
    IMU::ImuData const * getImuData() const {return imuData;};

    /**
    * @brief  Read-only access to the IMU object.
    * @return Reference to the IMU.
    */
    const IMU& getImu() const {return imu;};

    /**
    * @brief  Read-only access to the demand processor, which holds RC demands and receiver state.
    * @return Reference to the demand processor.
    */
    const DemandProcessor& getRc() const {return rc;};

    /**
    * @brief  Read-only access to the roll axis PIDF controller.
    * @return Reference to the roll PIDF.
    */
    const PIDF& getRollPIDF() const {return rollPIDF;};

    /**
    * @brief  Read-only access to the pitch axis PIDF controller.
    * @return Reference to the pitch PIDF.
    */
    const PIDF& getPitchPIDF() const {return pitchPIDF;};

    /**
    * @brief  Read-only access to the yaw axis PIDF controller.
    * @return Reference to the yaw PIDF.
    */
    const PIDF& getYawPIDF() const {return yawPIDF;};

    /**
    * @brief  Read-only access to the active airframe mixer.
    * @return Pointer to the active model.
    */
    const ModelBase* getModel() const {return myModel;};

    /**
    * @brief  Read-only access to the battery monitor.
    * @return Reference to the battery monitor.
    */
    const BatteryMonitor& getBatteryMonitor() const {return batteryMonitor;};

    /**
    * @brief  Read-only access to the persisted configuration (PID gains, rates, trims etc).
    * @return Reference to the configurator.
    */
    const Configurator& getConfig() const {return config;};

    /**
    * @brief  Latest GNSS sample received.
    * @return Reference to the last GNSS data; zeroed until the first sample arrives.
    */
    const GnssData& getLastGnss() const {return m_lastGnss;};

  private:   
    //Constants
    static constexpr int32_t ACRO_TRAINER_RECOVERY_RATE = static_cast<int32_t>(Config::ACRO_TRAINER_LEVEL_RATE * 100.0f);

    //Variables
    DemandProcessor::FlightState m_flightState = DemandProcessor::FlightState::CALIBRATE;
    DemandProcessor::FlightState m_lastFlightState = DemandProcessor::FlightState::CALIBRATE;
    IMU::ImuData*  imuData;
    ModelBase * myModel;
    GnssData m_lastGnss = {};

    //Methods
    void modelConfig();
    void doCalibrateState();
    void doDisarmedState();
    void doPassThroughState();
    void doRateState();
    void doLevelledState();
    void doFailSafeState();
    void doWifiApState();
    void doFaultedState();
    void doPropHangState();
    void doAcroTrainerState();
    void checkStateChange();
    void processPIDF(DemandProcessor::Demands * const demands);

    //Objects
    Led statusLed = Led(Config::ESP32S3.LED_ON_BOARD, Config::ESP32S3.SINK_ONBOARD_LED);
    LedNeopixel statusLedNeopixel = LedNeopixel(Config::ESP32S3.LED_ON_BOARD);
    Led externLed = Led(Config::ESP32S3.LED_EXTERNAL);
    

    DemandProcessor rc = DemandProcessor();
    BatteryMonitor batteryMonitor = BatteryMonitor(Config::ESP32S3.BATTERY_ADC);
    PIDF pitchPIDF = PIDF();
    PIDF yawPIDF = PIDF();
    PIDF rollPIDF = PIDF();
    IMU imu = IMU();  
    Configurator config;  
    TelemetryManager telemetryManager;
    GnssDriver gnss;
};