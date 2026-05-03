# ESP32_SoftWire

ESP32 fast bit-bang I2C library for Arduino, drop in replacement for Wire.h

The library reaches up to 3 MHz I2C clock speed. No fancy bits and bobs: no timeouts, no clock stretching, blocking only... Made for fast IMU sensor reading were an occasional missed read does not matter, but bus hangups do matter.

Tested on ESP32, probably works on ESP32-S3 and ESP32-C3, might work on other ESP32 variants.

Limitation: pins 0-31 only

## Background

As of December 2023 the official [arduino-esp32 Wire](https://github.com/espressif/arduino-esp32/tree/master/libraries/Wire) library has some nasty bugs. In particular, there is a *1 second* timeout which hangs the bus on a bad read, and this timeout can not be lowered without changing the source code, see https://github.com/espressif/esp-idf/issues/4999 and https://github.com/espressif/arduino-esp32/issues/5934. So occasionally (a couple times per minute) 1000 samples are missed when you are reading a sensor at 1 kHz. 
