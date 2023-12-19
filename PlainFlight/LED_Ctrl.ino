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

#include "Flight_Ctrl.h"
#include "defines.h"

//Set the actual output pin we intend to use for the LED
#ifdef USE_LED_BUILTIN
  #define LED_OUTPUT  LED_BUILTIN
#else
  #define LED_OUTPUT  LED_PIN
#endif

typedef struct
{
  const bool state;
  const uint32_t duration;
}Led_Bit;

typedef struct
{
  const Led_Bit* led;
  const uint32_t size;
}ptrSeq;

#define NUM_SEQUENCES 6

#if defined(SINK_LED)
  //Here we define the flash sequences for each flight mode/state
  static const Led_Bit ledSeqDisarmed[2]     = {{false, 1500U}, {true, 1500}};     //1 second flash for disarmed
  static const Led_Bit ledSeqPassThrough[2]  = {{false, 150U},{true, 1850U}};      //1 quick flash for pass through
  static const Led_Bit ledSeqRateMode[4]     = {{false, 150U},{true, 150U},{false, 150U},{true, 1550U}};    //2 quick flashes for rate mode
  static const Led_Bit ledSeqLevelledMode[6] = {{false, 150U},{true, 150U},{false, 150U},{true, 150U},{false, 150U},{true, 1250U}};    //3 flashes for levelled mode
  static const Led_Bit ledSeqFailsafe[2]     = {{false, 150U},{true, 150U}};       //constant quick flashing
  static const Led_Bit ledSeqUncalibrated[2] = {{false, 2850U},{true, 150U}};       //long flash
#else //Source LED
  static const Led_Bit ledSeqDisarmed[2]     = {{true, 1000U}, {false, 1000}};     //1 second flash for disarmed
  static const Led_Bit ledSeqPassThrough[2]  = {{true, 150},{false, 1850}};        //1 quick flash for pass through
  static const Led_Bit ledSeqRateMode[4]     = {{true, 150},{false, 150},{true, 150},{false, 1550}};    //2 quick flashes for rate mode
  static const Led_Bit ledSeqLevelledMode[6] = {{true, 150},{false, 150},{true, 150},{false, 150},{true, 150},{false, 1250}};    //3 flashes for levelled mode
  static const Led_Bit ledSeqFailsafe[2]     = {{true, 150},{false, 150}};         //constant quick flashing
  static const Led_Bit ledSeqUncalibrated[2] = {{true, 2850U},{false, 150U}};       //long flash  //TODO - add calibrate flash to instructions
#endif

//Define an array of sequences and their size
static const ptrSeq sequences[NUM_SEQUENCES] = 
{
  {&ledSeqDisarmed[0],    sizeof(ledSeqDisarmed)/8U},
  {&ledSeqPassThrough[0], sizeof(ledSeqPassThrough)/8U},
  {&ledSeqRateMode[0],    sizeof(ledSeqRateMode)/8U},
  {&ledSeqLevelledMode[0],sizeof(ledSeqLevelledMode)/8U},
  {&ledSeqFailsafe[0],    sizeof(ledSeqFailsafe)/8U},
  {&ledSeqUncalibrated[0],sizeof(ledSeqUncalibrated)/8U},
};

//Arduino requires this declaration here for typedef's to work in function prototypes
void playLedSequence(states currentState);


/*
* DESCRIPTION: Set the port pin for LED output.
*/
void initLED(void)
{
  pinMode(LED_OUTPUT, OUTPUT);
}


/*
* DESCRIPTION: Plays an LED flash sequence depending upon the flight state passed in.
*/
void playLedSequence(states currentState)
{
  static uint32_t idx = 0U;
  static uint64_t nextSeqTime = 0U;
  static bool sequenceFinished = false;
  static states playSequence;
  static const uint32_t numSequences = NUM_SEQUENCES; 
  uint64_t nowTime = millis();

  if (sequenceFinished)
  {
    //Only change sequence once last sequence has finished
    sequenceFinished = false;
    //We should never get an error but just good practice to check and handle... 
    playSequence = (currentState < numSequences) ? currentState: state_failsafe;
  }

  //Play the LED flash sequence
  if (nowTime >=  nextSeqTime)
  {
    digitalWrite(LED_OUTPUT, sequences[playSequence].led[idx].state);
    nextSeqTime = nowTime + (uint64_t)sequences[playSequence].led[idx].duration;
    idx = (++idx >= sequences[playSequence].size) ? 0U: idx;
    sequenceFinished = (0U == idx) ? true:false;
  }
}
