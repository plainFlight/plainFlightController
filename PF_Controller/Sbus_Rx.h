#ifndef SBUS_RX_H
#define SBUS_RX_H

//#define USE_ALL_16_CHANNELS

#define NUM_CH          16

typedef struct {
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  uint32_t ch[NUM_CH];
}Sbus_Data;

Sbus_Data rxData;// = {true, true, false, false, NUM_CH, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

#endif