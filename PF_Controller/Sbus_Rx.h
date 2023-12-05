/*
* MIT License
*
* Copyright (c) 2023 plainFlight
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
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