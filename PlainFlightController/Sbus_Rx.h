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

#ifndef SBUS_RX_H
#define SBUS_RX_H

//Uncomment following line if you wish to decode and use all 18 channels of Sbus.
//#define USE_ALL_18_CHANNELS

#define SBUS_UART   &Serial0  //Seed Studio XIAO ESP32-C3/S3
#define NUM_CH      16

typedef struct 
{
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  uint32_t ch[NUM_CH];
}Sbus_Data;

Sbus_Data rxData;

#endif