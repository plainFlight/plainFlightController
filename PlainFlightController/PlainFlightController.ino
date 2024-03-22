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

#include "Defines.h"
#include "Flight_Ctrl.h"

void setup(void) 
{
  Serial.begin(USB_BAUD);
  ledBuiltIn.begin();
  ledExternal.begin();
  initActuators();
  initSbusRx();
  initIMU();
  initBatteryMonitor();  
}


void loop(void) 
{
  loopRateControl();
  flightControl();
}



