#include "LED_Ctrl.h"

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#define NUM_SEQUENCES 7

//Here we define the flash sequences for each flight mode/state
  static const Led_Bit ledSeqDisarmed[2]     = {{false, 1500U}, {true, 1500}};     //1.5 second flash for disarmed
  static const Led_Bit ledSeqPassThrough[2]  = {{false, 150U},{true, 1850U}};      //1 quick flash for pass through
  static const Led_Bit ledSeqRateMode[4]     = {{false, 150U},{true, 150U},{false, 150U},{true, 1550U}};    //2 quick flashes for rate mode
  static const Led_Bit ledSeqLevelledMode[6] = {{false, 150U},{true, 150U},{false, 150U},{true, 150U},{false, 150U},{true, 1250U}};    //3 flashes for levelled mode
  static const Led_Bit ledSeqFailsafe[2]     = {{false, 150U},{true, 150U}};       //constant quick flashing
  static const Led_Bit ledSeqCalibrating[2]  = {{false, 2850U},{true, 150U}};      //long flash
  static const Led_Bit ledSeqOn[2]           = {{false, 999},{true, 1}};

static const Ptr_Seq sequences[NUM_SEQUENCES] = 
{
  {&ledSeqDisarmed[0],    sizeof(ledSeqDisarmed)/8U},
  {&ledSeqPassThrough[0], sizeof(ledSeqPassThrough)/8U},
  {&ledSeqRateMode[0],    sizeof(ledSeqRateMode)/8U},
  {&ledSeqLevelledMode[0],sizeof(ledSeqLevelledMode)/8U},
  {&ledSeqFailsafe[0],    sizeof(ledSeqFailsafe)/8U},
  {&ledSeqCalibrating[0], sizeof(ledSeqCalibrating)/8U},
  {&ledSeqOn[0],          sizeof(ledSeqCalibrating)/8U},
};


/*
* DESCRIPTION: Instantiates an LED output.
*/
LED::LED(uint32_t ledPin, bool sinkLed)
{
  this->ledPin = ledPin;  
  this->sinkLed = sinkLed;
  this->idx = 0U;
  this->nextSeqTime = 0U;
  this->sequenceFinished = false;
  this->playSequence = 0;
  this->numSequences = NUM_SEQUENCES; 
  
}


/*
* DESCRIPTION: Set the port pin for LED output.
*/
void LED::begin(void)
{
  pinMode(this->ledPin, OUTPUT);
  this->nowTime = millis();
}


/*
* DESCRIPTION: Plays an LED flash sequence depending upon the flight state passed in.
*/
void LED::run(uint32_t sequence)
{
   this->nowTime = millis();

  if (this->sequenceFinished)
  {
    //Only change sequence once last sequence has finished
    this->sequenceFinished = false;
    //We should never get an error but just good practice to check and handle... 
    playSequence = (sequence < numSequences) ? sequence: 0;
  }

  //Play the LED flash sequence
  if (this->nowTime >=  this->nextSeqTime)
  {
    bool state = (this->sinkLed) ? sequences[this->playSequence].led[this->idx].state : !sequences[this->playSequence].led[this->idx].state;
    digitalWrite(this->ledPin, state); 
    this->nextSeqTime = this->nowTime + (uint64_t)sequences[this->playSequence].led[this->idx].duration;
    this->idx = (++this->idx >= sequences[this->playSequence].size) ? 0U: this->idx;
    this->sequenceFinished = (0U == this->idx) ? true:false;
  }
}