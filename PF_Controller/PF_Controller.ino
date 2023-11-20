#include "Defines.h"

void setup(void) 
{
  Serial.begin(USB_BAUD);
  initLED();
  initActuators();
  initRadioControl();
  initIMU();
  initBatteryMonitor();  
}


void loop(void) 
{
  loopRateControl();
  flightControl();
}



