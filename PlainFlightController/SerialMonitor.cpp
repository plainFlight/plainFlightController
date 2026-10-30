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
* @file   SerialMonitor.cpp
* @brief  This class implements DEBUG outputs via the serial console port.
*/

#include "SerialMonitor.hpp"
#include "FlightControl.hpp"


/**
* @brief  Called once at the end of FlightControl::begin().
*/
void
SerialMonitor::begin(const FlightControl& /*fc*/)
{
}


/**
* @brief  Called at the end of every loop.
*/
void
SerialMonitor::update(const FlightControl& /*fc*/)
{
}
