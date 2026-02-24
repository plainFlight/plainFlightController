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
#include "BoardConfig.hpp"
#include "RxBase.hpp"
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

  private:   
    //Constants
    static constexpr int32_t ACRO_TRAINER_RECOVERY_RATE = static_cast<int32_t>(Config::ACRO_TRAINER_LEVEL_RATE * 100.0f);

    //Variables
    DemandProcessor::FlightState m_flightState = DemandProcessor::FlightState::CALIBRATE;
    DemandProcessor::FlightState m_lastFlightState = DemandProcessor::FlightState::CALIBRATE;
    IMU::ImuData*  imuData;
    ModelBase * myModel;

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
    Led statusLed = Led(BoardConfig::LED_ONBOARD);
    LedNeopixel statusLedNeopixel = LedNeopixel(BoardConfig::LED_ONBOARD);
    Led externLed = Led(BoardConfig::EXT_LED_PIN);
    

    DemandProcessor rc = DemandProcessor();
    BatteryMonitor batteryMonitor = BatteryMonitor(BoardConfig::BATT_ADC_PIN);
    PIDF pitchPIDF = PIDF();
    PIDF yawPIDF = PIDF();
    PIDF rollPIDF = PIDF();
    IMU imu = IMU();  
    Configurator config;  
};