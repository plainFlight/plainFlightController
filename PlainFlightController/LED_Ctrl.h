#ifndef LED_CTRL_H
#define LED_CTRL_H

#include <inttypes.h>

typedef struct
{
  const bool state;
  const uint32_t duration;
}Led_Bit;

typedef struct
{
  const Led_Bit* led;
  const uint32_t size;
}Ptr_Seq;

typedef enum Sink_Source
{
  SINK = true,
  SOURCE = false,
};

class LED 
{
  private: 
    uint32_t ledPin;
    bool sinkLed;
    uint32_t idx = 0U;
    uint64_t nextSeqTime = 0U;
    bool sequenceFinished = false;
    uint32_t playSequence;
    uint32_t numSequences; 
    uint64_t nowTime;

  public:
    LED(uint32_t ledPin, bool sinkLed);
    void begin(void);
    void run(uint32_t sequence);
};

#endif