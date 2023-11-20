#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

typedef enum states
{
  state_disarmed,
  state_pass_through,
  state_rate,
  state_auto_level,
  state_failsafe,
};

enum
{
  rate_gain = 0,
  levelled_gain,
}gain_bank;

#endif