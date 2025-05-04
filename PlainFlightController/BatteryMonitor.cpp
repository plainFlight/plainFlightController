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
* @file   BatteryMonitor.cpp
* @brief  This class contains methods that all clases may call upon
*/

#include "BatteryMonitor.hpp"


/**
* @brief    Battery monitor constructor.  
* @param    IO pin used to monitor battery voltage.
*/ 
BatteryMonitor::BatteryMonitor(const uint8_t monitorPin)
{
  m_monitorPin = monitorPin;
  pinMode(m_monitorPin, INPUT);
}


/**
* @brief  Sets ADC pin for battery monitor, primes the battery voltage filter, then detects number of cells in flight pack/battery.
* @param  batteryScaler Value by wich the ADC is mutliplied by.
* @note   batteryVoltage initilised to 8.4V to help speed filter prime.
*/
void 
BatteryMonitor::begin(const float batteryScaler)
{
  m_batteryScaler = batteryScaler;

  for (uint32_t i=0U; i<static_cast<uint32_t>(FILTER_DIVISOR); i++)
  {
    operate();
  }

  setNumberCells();
}


/**
* @brief Used at initialisation to determine number of lipo cells connected to ESC(s).
* @note Cell voltage used as this simplifies low battery detection when flight controller is used with varying cell counts.
*/
void 
BatteryMonitor::setNumberCells()
{
  if (m_batteryVoltage > MIN_3S_VOLTAGE)
  {
    m_numberCells = 3U;
  }
  else if (m_batteryVoltage > MIN_2S_VOLTAGE)
  {
    m_numberCells = 2U;
  }
  else
  {
    m_numberCells = 1U;
  }

  if constexpr(Config::DEBUG_BATTERY_MONITOR)
  {
    Serial.print("Cells detected: ");
    Serial.println(m_numberCells);
  }
}


/**
* @brief  Main routine of BatteryMonitor, must be called regularly.  
* @param  batteryScaler Value by wich the ADC is mutliplied by.
*/ 
void 
BatteryMonitor::operate()
{
  const float rawVoltage = static_cast<float>(analogRead(m_monitorPin)) * m_batteryScaler;
  //Simple weighted filter...
  m_batteryVoltage = ((m_batteryVoltage * WEIGHT_OLD) + (rawVoltage * WEIGHT_NEW)) / FILTER_DIVISOR;
}


/**
* @brief    gets the current battery voltage 
* @return   Floating point representing current battery voltage
*/ 
float 
BatteryMonitor::getVoltage()
{
  return m_batteryVoltage;
}


/**
* @brief    sets the battery voltage scaler
* @return   Floating point representing battery scaler
*/ 
void 
BatteryMonitor::setVoltageScaler(const float newVoltageScaler)
{
  m_batteryScaler = newVoltageScaler;
}


/**
* @brief    Outputs debug data
*/ 
void
BatteryMonitor::debug()
{
  Serial.print("ADC: ");
  Serial.print(analogRead(m_monitorPin));
  Serial.print(", Cells: ");
  Serial.print(m_numberCells);
  Serial.print(", Scaler:");
  Serial.print(m_batteryScaler, 5);
  Serial.print(", volts:");
  Serial.println(m_batteryVoltage);
}


/**
* @brief  Limits throttle when battery voltage falls below LOW_CELL_VOLTAGE.
* @param  throttleDemand The demanded thottle from the transmitter.
* @param  throttleHigh   When true the throttle stick is high.
* @param  minThrottle    The minimum possible thottle demand from the transmitter.
* @note   Throttle is actively managed based upon the battery voltage and throttle demand - Simple but effective solution.
*/
int32_t 
BatteryMonitor::limitThrottle(const int32_t throttleDemand, const bool throttleHigh, const int32_t minThrottle)
{
  const float cellVoltage = m_batteryVoltage / static_cast<float>(m_numberCells);

  if (BatteryMonitor::MIN_CELL_VOLTAGE > cellVoltage)
  {
    return minThrottle;
  }
  else
  {
    if ((BatteryMonitor::LOW_CELL_VOLTAGE > cellVoltage) && throttleHigh)
    {
      //Throttled up and low voltage so actively limit the throttle by mapping the current voltage to current demand.
      return map32(static_cast<int32_t>(cellVoltage*10000.0f), MIN_CELL_VOLTAGE_x10000, LOW_CELL_VOLTAGE_x10000, minThrottle, throttleDemand);
    }
  }

  return throttleDemand;
}