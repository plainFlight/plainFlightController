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



class LedSequences
{
  private: 
    struct Colour
    {
      uint8_t red;
      uint8_t green;
      uint8_t blue;
    };

    //Constants
    static constexpr Colour RED       = {255, 0, 0};
    static constexpr Colour GREEN     = {0, 255, 0};
    static constexpr Colour BLUE      = {0, 0, 255};
    static constexpr Colour PURPLE    = {255, 0, 255};
    static constexpr Colour YELLOW    = {255, 255, 0};
    static constexpr Colour WHITE     = {255, 255, 255};
    static constexpr Colour NO_COLOUR = {0, 0, 0};

    struct LedState 
    {
      uint32_t duration;
      Colour colour;
      bool ledState;
    };

    struct PtrSeq
    {
      const LedState* led;
      const uint32_t size;
    };    

    static constexpr LedState PASS_THROUGH[2]       = {{125U, RED, true},{1000U, NO_COLOUR, false}};  //PassThrough - 1 quick flash
    static constexpr LedState RATE[4]               = {{125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{1000U, NO_COLOUR, false}};   //Rate - 2 quick flashes 
    static constexpr LedState SELF_LEVELLED[6]      = {{125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{1000U, NO_COLOUR, false}};   //Self Levelled - 3 quick flashes  
    static constexpr LedState ACRO_TRAINER[8]       = {{125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{1000U, NO_COLOUR, false}};   //Acro trainer - 4 quick flashes
    static constexpr LedState WAIT_DISARM[10]       = {{125U, BLUE, true},{125U, NO_COLOUR, false},
                                                      {125U, BLUE, true},{125U, NO_COLOUR, false},
                                                      {125U, BLUE, true},{125U, NO_COLOUR, false},
                                                      {125U, BLUE, true},{125U, NO_COLOUR, false},
                                                      {125U, BLUE, true},{1000U, NO_COLOUR, false}};   //Wait To Disarm - 5 quick flashes    
    static constexpr LedState WIFI_AP[12]           = {{125U, YELLOW, true},{125U, NO_COLOUR, false},
                                                      {125U, YELLOW, true},{125U, NO_COLOUR, false},
                                                      {125U, YELLOW, true},{125U, NO_COLOUR, false},
                                                      {125U, YELLOW, true},{125U, NO_COLOUR, false},
                                                      {125U, YELLOW, true},{125U, NO_COLOUR, false},
                                                      {125U, YELLOW, true},{1000U, NO_COLOUR, false}};   //Wifi AP - 6 quick flashes
    static constexpr LedState FAULT[14]             = {{125U, PURPLE, true},{125U, NO_COLOUR, false},
                                                      {125U, PURPLE, true},{125U, NO_COLOUR, false},
                                                      {125U, PURPLE, true},{125U, NO_COLOUR, false},
                                                      {125U, PURPLE, true},{125U, NO_COLOUR, false},
                                                      {125U, PURPLE, true},{125U, NO_COLOUR, false},
                                                      {125U, PURPLE, true},{125U, NO_COLOUR, false},
                                                      {125U, PURPLE, true},{1000U, NO_COLOUR, false}};   //Fault - 7 quick flashes
    static constexpr LedState PROP_HANG[16]         = {{125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{125U, NO_COLOUR, false},
                                                      {125U, RED, true},{1000U, NO_COLOUR, false}};     //Prop hang - 8 quick flashes
    static constexpr LedState DISARMED[2]           = {{1500U, GREEN, true}, {1500, NO_COLOUR, false}}; //Disarmed - 1.5 second flash for disarmed
    static constexpr LedState CALIBRATION[2]        = {{250U, WHITE, true}, {250, NO_COLOUR, false}};   //Calibrating - 250ms second flash 
    static constexpr LedState FAILSAFE[2]           = {{125U, PURPLE, true},{125U, NO_COLOUR, false}};  //Failsafe constant quick flashing  

    

  protected:
    static constexpr uint32_t NUM_SEQUENCES = 11U;

    //Note: Oder of array must match FlightState...
    static constexpr PtrSeq sequences[NUM_SEQUENCES] = 
    {
      {&PASS_THROUGH[0],  sizeof(PASS_THROUGH)/8U},
      {&RATE[0],          sizeof(RATE)/8U},
      {&SELF_LEVELLED[0], sizeof(SELF_LEVELLED)/8U},
      {&ACRO_TRAINER[0],  sizeof(ACRO_TRAINER)/8U}, 
      {&PROP_HANG[0],     sizeof(PROP_HANG)/8U},
      {&WAIT_DISARM[0],   sizeof(WAIT_DISARM)/8U},
      {&WIFI_AP[0],       sizeof(WIFI_AP)/8U},
      {&FAULT[0],         sizeof(FAULT)/8U},
      {&CALIBRATION[0],   sizeof(CALIBRATION)/8U},
      {&DISARMED[0],      sizeof(DISARMED)/8U},           
      {&FAILSAFE[0],      sizeof(FAILSAFE)/8U},     
    };

    public:
      LedSequences(){};
      ~LedSequences(){};
};


class Led : public LedSequences
{
  private:

  protected:   

    //Variables
    uint32_t m_ledPin;
    bool m_sinkLed;
    uint32_t m_idx = 0U;
    bool m_sequenceFinished = false;
    uint32_t m_playSequence = 0U;

    CTimer sequenceTimer = CTimer(0);

  public:
    Led(uint32_t ledPin)
    {
      m_ledPin = ledPin;
    }

    ~Led(){};

    void begin()
    {
      pinMode(m_ledPin, OUTPUT);  //No actually needed for neopixel
      sequenceTimer.set(0U);      //Force timer to run
    }

    void operate(uint32_t sequence)
    {
      if (m_sequenceFinished)
      {
        //Only change sequence once last sequence has finished
        m_sequenceFinished = false;
        //We should never get an error but just good practice to check and handle... 
        m_playSequence = (sequence < NUM_SEQUENCES) ? sequence: 0U;
      }

      //Play the LED flash sequence
      if (CTimer::State::EXPIRED == sequenceTimer.getState())
      {
        setLed();
        sequenceTimer.set(sequences[m_playSequence].led[m_idx].duration);
        m_idx = (++m_idx >= sequences[m_playSequence].size) ? 0U: m_idx;
        m_sequenceFinished = (0U == m_idx) ? true:false;
      }
    }

    virtual void setLed()
    {
      digitalWrite(m_ledPin, sequences[m_playSequence].led[m_idx].ledState);
    }
};



class LedNeopixel : public Led
{

  public:
    LedNeopixel(uint8_t ledPin) : Led(ledPin){};
    ~LedNeopixel(){};

    virtual void setLed() override
    {
      rgbLedWrite(m_ledPin, 
                  sequences[m_playSequence].led[m_idx].colour.red, 
                  sequences[m_playSequence].led[m_idx].colour.green,
                  sequences[m_playSequence].led[m_idx].colour.blue);
    }
};