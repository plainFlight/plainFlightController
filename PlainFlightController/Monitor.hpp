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
* @file   Monitor.hpp
* @brief  This is the base class for anything that reports on the internal state of the flight controller.
*/

#pragma once

#include "Config.hpp"

class FlightControl;

/**
* @class  Monitor
* @brief  Base class for anything that reports on the flight controller: serial output, logging.
* @note   Monitors are installed in setup() before FlightControl::begin(). A derived class
*         overrides only the methods it needs.
*/
class Monitor
{
  public:
    virtual ~Monitor() = default;

    /** @brief  Called once at the end of FlightControl::begin(). */
    virtual void begin(const FlightControl& /*fc*/) {}

    /** @brief  Called at the end of every loop, after the outputs are written. */
    virtual void update(const FlightControl& /*fc*/) {}

    /** @brief  Adds a monitor to the end of the list. Install each monitor once only. */
    static void install(Monitor& monitor)
    {
      Monitor** link = &s_first;
      while (*link != nullptr)
      {
        link = &((*link)->m_next);
      }
      *link = &monitor;
    }

    static void beginAll(const FlightControl& fc)
    {
      if constexpr (Config::USE_MONITOR)
      {
        for (Monitor* m = s_first; m != nullptr; m = m->m_next)
        {
          m->begin(fc);
        }
      }
    }

    static void updateAll(const FlightControl& fc)
    {
      if constexpr (Config::USE_MONITOR)
      {
        for (Monitor* m = s_first; m != nullptr; m = m->m_next)
        {
          m->update(fc);
        }
      }
    }

  private:
    Monitor* m_next = nullptr;
    inline static Monitor* s_first = nullptr;
};
