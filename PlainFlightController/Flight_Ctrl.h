/* 
* Copyright (c) 2023,2024 P.Cook (alias 'plainFlight')
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

#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

#include "LED_Ctrl.h"

#define FAILSAFE_ROLL_ANGLE_x100    (int32_t)(FAILSAFE_ROLL_ANGLE * 100.0f)
#define FAILSAFE_PITCH_ANGLE_x100   (int32_t)(FAILSAFE_ROLL_ANGLE * 100.0f)

typedef enum states
{
  state_disarmed = 0U,
  state_pass_through,
  state_rate,
  state_auto_level,
  state_failsafe,
  state_calibrating,
};

LED ledBuiltIn(LED_BUILTIN, SINK);
#if defined(SINK_EXTERNAL_LED)
  LED ledExternal(EXT_LED_PIN, SINK);
#else
  LED ledExternal(EXT_LED_PIN, SOURCE);
#endif

#endif