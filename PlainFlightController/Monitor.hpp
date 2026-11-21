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
#include <cstdarg>
#include <cstdio>

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

    /** @brief  Called once at the end of FlightControl::begin().
    * @param  fc  The flight controller being monitored.
    */
    virtual void begin(const FlightControl& /*fc*/) {}

    /** @brief  Called at the end of every loop, after the outputs are written. 
     * @param  fc  The flight controller being monitored.
    */
    virtual void update(const FlightControl& /*fc*/) {}

    /** @brief  Called for each event raised with event() or eventf().
     * @param  source  The source that raised the event.
     * @param  text    The message.
    */
    virtual void onEvent(MonitorSource /*source*/, const char* /*text*/) {}

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

    /** @brief  Calls begin on all monitor instances. 
     * @param  fc  The flight controller being monitored.
    */
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

    /** @brief  Calls update on all monitor instances. 
     * @param  fc  The flight controller being monitored.
    */
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

    /** @brief  Passes an event to every installed monitor.
     * @param  source  The source raising the event.
     * @param  text    The message.
    */
    static void event(MonitorSource source, const char* text)
    {
      if constexpr (Config::USE_MONITOR)
      {
        for (Monitor* m = s_first; m != nullptr; m = m->m_next)
        {
          m->onEvent(source, text);
        }
      }
    }

    /** @brief  printf-style event. Text longer than EVENT_TEXT_SIZE - 1 is truncated.
     * @param  source  The source raising the event.
     * @param  format  printf format string, followed by its values.
    */
    __attribute__((format(printf, 2, 3)))
    static void eventf(MonitorSource source, const char* format, ...)
    {
      if constexpr (Config::USE_MONITOR)
      {
        char text[EVENT_TEXT_SIZE];
        va_list args;
        va_start(args, format);
        vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        event(source, text);
      }
    }

  private:
    static constexpr size_t EVENT_TEXT_SIZE = 96U;  // eventf() buffer, on the stack during a call
    Monitor* m_next = nullptr;
    inline static Monitor* s_first = nullptr;
};
