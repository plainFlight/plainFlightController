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
    static constexpr uint32_t RX_RATE_WINDOW_MS = 1000U;   // RX frame rate measured over 1 s

    bool printDue();
    void printLoopRate(const FlightControl& fc);
    void printRcData(const FlightControl& fc);
    void printBattery(const FlightControl& fc);
    void printMadgwick(const FlightControl& fc);
    void printImu(const FlightControl& fc);
    void printOutputs(const FlightControl& fc);
    void printGps(const FlightControl& fc);
    void printRx(const FlightControl& fc);

    uint32_t m_nextPrintMs = 0U;
    float    m_avCycleTimeUs = 0.0f;       // LOOP_RATE: running average, updated every loop
    bool     m_lastImuOk = true;           // IMU: report a read error once, on the change
    uint8_t  m_lastGnssSecond = 0U;        // GPS: time of the last fix printed
    uint16_t m_lastGnssMs = 0U;
    uint32_t m_rxFrames = 0U;              // RX: frames received since the last print
    uint32_t m_rxWindowStartMs = 0U;       // RX: time the current count started
    uint32_t m_rxHz = 0U;                  // RX: frame rate from the last full second
};
