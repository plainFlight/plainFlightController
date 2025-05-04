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
* @file   LED.hpp
* @brief  This class handles LED flash sequences.
*/
#pragma once

#include <inttypes.h>
#include <Arduino.h>
#include "Timer.hpp"



/**
* @class  LED
*/
 
class LED 
{
  private: 
    struct LedBit
    {
      const uint32_t duration;
      const bool state;
    };

    struct PtrSeq
    {
      const LedBit* led;
      const uint32_t size;
    };

    //Constants  
    static constexpr uint32_t NUM_SEQUENCES = 11U;
    //Here we define the flash sequences for each flight mode/state
    static constexpr LedBit ledSeqPassThrough[2]  = {{125U, true},{1000U, false}};  //PassThrough - 1 quick flash 
    static constexpr LedBit ledSeqRate[4]         = {{125U, true},{125U, false},
                                                    {125U, true},{1000U, false}};   //Rate - 2 quick flashes 
    static constexpr LedBit ledSeqSelfLevelled[6] = {{125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{1000U, false}};   //Self Levelled - 3 quick flashes  
    static constexpr LedBit ledSeqAcroTrainer[8]  = {{125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{1000U, false}};   //Acro trainer - 4 quick flashes
    static constexpr LedBit ledSeqWaitDisarm[10]  = {{125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{1000U, false}};   //Wait To Disarm - 5 quick flashes    
    static constexpr LedBit ledSeqWifiAp[12]      = {{125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{1000U, false}};   //Wifi AP - 6 quick flashes
    static constexpr LedBit ledSeqFault[14]       = {{125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{1000U, false}};   //Fault - 7 quick flashes
    static constexpr LedBit ledSeqPropHang[16]    = {{125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{125U, false},
                                                    {125U, true},{1000U, false}};   //Prop hang - 8 quick flashes
    static constexpr LedBit ledSeqDisarmed[2]     = {{1500U, true}, {1500, false}}; //Disarmed - 1.5 second flash for disarmed
    static constexpr LedBit ledSeqCalibrating[2]  = {{250U, true}, {250, false}};   //Calibrating - 250ms second flash 
    static constexpr LedBit ledSeqFailsafe[2]     = {{125U, true},{125U, false}};   //Failsafe constant quick flashing       

    //Note: Oder of array must match FlightState...
    static constexpr PtrSeq sequences[NUM_SEQUENCES] = 
    {
      {&ledSeqPassThrough[0], sizeof(ledSeqPassThrough)/8U},
      {&ledSeqRate[0],        sizeof(ledSeqRate)/8U},
      {&ledSeqSelfLevelled[0],sizeof(ledSeqSelfLevelled)/8U},
      {&ledSeqAcroTrainer[0], sizeof(ledSeqAcroTrainer)/8U}, 
      {&ledSeqPropHang[0],    sizeof(ledSeqPropHang)/8U},
      {&ledSeqWaitDisarm[0],  sizeof(ledSeqWaitDisarm)/8U},
      {&ledSeqWifiAp[0],      sizeof(ledSeqWifiAp)/8U},
      {&ledSeqFault[0],       sizeof(ledSeqFault)/8U},
      {&ledSeqCalibrating[0], sizeof(ledSeqCalibrating)/8U},
      {&ledSeqDisarmed[0],    sizeof(ledSeqDisarmed)/8U},           
      {&ledSeqFailsafe[0],    sizeof(ledSeqFailsafe)/8U},     
    };

    //Variables
    uint32_t m_ledPin;
    bool m_sinkLed;
    uint32_t m_idx = 0U;
    bool m_sequenceFinished = false;
    uint32_t m_playSequence = 0U;
    uint32_t m_numSequences = NUM_SEQUENCES; 
    //Objects
    CTimer sequenceTimer = CTimer(0);

  public:
    enum class SinkSource : bool
    {
      SINK   = true,
      SOURCE = false,
    };

    LED(uint32_t ledPin, SinkSource sinkSource);
    void begin();
    void operate(uint32_t sequence);
};
