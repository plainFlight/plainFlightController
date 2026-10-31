/* 
* Original File Author: D. Gamble (Github: Cyberslug)
*
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
* @file   SerialMonitor.hpp
* @brief  This class implements DEBUG outputs via the serial console port.
*/
#pragma once

#include <cstdint>
#include "Monitor.hpp"

class SerialMonitor : public Monitor
{
  public:
    void begin(const FlightControl& fc) override;
    void update(const FlightControl& fc) override;

  private:
    static constexpr MonitorSource SOURCE = Config::MONITOR_SOURCE;
    static constexpr uint32_t PRINT_PERIOD_MS = 100U;   // 10 Hz output for every source

    bool printDue();
    void printLoopRate(const FlightControl& fc);
    void printRcData(const FlightControl& fc);
    void printBattery(const FlightControl& fc);
    void printMadgwick(const FlightControl& fc);

    uint32_t m_nextPrintMs = 0U;
    float    m_avCycleTimeUs = 0.0f;       // LOOP_RATE: running average, updated every loop
};
