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
* @file   BatteryMonitor.hpp
* @brief  This class contains methods that all clases may call upon
*/
#pragma once

#include <inttypes.h>
#include <Arduino.h>
#include "Utilities.hpp"
#include "Config.hpp"



/*
* To measure battery voltage a potential divider is used with the following values:
* R1=61.9K, R2=17.4K... 15.04V max input = 3.3V out, ...12.6V in = 2.772V out
* Note: Wise to add 10uF capacitor across R2 but not vital. You may need to increase weighted filter values if not fitted.
*   
*                        ______ Vin... +ve of flight battery. 3s Lipo max or you will damage the ESP32!
*                      _|_
*                     |   |
*                     |   |  R1 = 61.9K Ohms
*                     |___|
*                _______|______ Vout... to BATT_ADC_PIN
*               |      _|_
*               |     |   |
*    10uF 25V __|__   |   |  R2 = 17.4K Ohms
*             _____   |___|
*               |       |
*               |_______|______ Gnd
*
* For the above resistor values and with ESP32S3 ADC resolution of 12bits; therefore 30V/4096 = 7.32mV per bit.
* If you cannot get the exact values shown find something similar and adjust ADC_MULTIPLIER to suit your values.
*/


/**
 * @class BatteryMonitor
 */
 
class BatteryMonitor : public Utilities
{
  public:
    static constexpr float MIN_CELL_VOLTAGE   = 3.3f;     
    static constexpr float LOW_CELL_VOLTAGE   = 3.4f;     //The per cell voltage threshold where we start to rduce throttle rpm's to indicate low voltage
    static constexpr int32_t MIN_CELL_VOLTAGE_x10000 = static_cast<int32_t>(BatteryMonitor::MIN_CELL_VOLTAGE * 10000.0f);
    static constexpr int32_t LOW_CELL_VOLTAGE_x10000 = static_cast<int32_t>(BatteryMonitor::LOW_CELL_VOLTAGE * 10000.0f);

    BatteryMonitor(const uint8_t monitorPin);
    void begin(const float batteryScaler);
    void operate();
    float getVoltage();
    void setVoltageScaler(const float newVoltageScaler);
    void debug();
    int32_t limitThrottle(const int32_t throttleDemand, const bool throttleHigh, const int32_t minMotorTicks);
    void setNumberCells();

  private: 
    static constexpr float WEIGHT_NEW = 1.0f;
    static constexpr float WEIGHT_OLD = 99.0f;
    static constexpr float FILTER_DIVISOR = (WEIGHT_NEW + WEIGHT_OLD);   
    
    static constexpr float MIN_3S_VOLTAGE     = 9.9f;     //9.9V
    static constexpr float MIN_2S_VOLTAGE     = 6.6f;     //6.6V
    static constexpr float MIN_1S_VOLTAGE     = 3.3f;     //3.3V
    static constexpr float MAX_1S_VOLTAGE     = 4.2f;     //3.3V

    float m_batteryVoltage = MAX_1S_VOLTAGE * 2.0f;  //Default it to 8.4V to speed initial cell detection at power on
    uint32_t m_numberCells;
    uint8_t m_monitorPin;
    float m_batteryScaler;
};