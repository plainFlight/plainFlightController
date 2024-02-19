![Logo](https://github.com/plainFlight/plainFlightController/blob/main/assets/images/PlainFlight%20Logo%20Large.PNG)
# Overview
PlainFlight stabilisation software is for the RC pilot who wants to get the most from their model, needs to master an unstable aircraft, or simply counteract environmental conditions for a more enjoyable flight.

Originally created as a home project for fixed wing model aircraft and with the intent of trying to replicate the adaptability and ease of build that the [MultiWii](https://code.google.com/archive/p/multiwii/) project had from many years ago. It quickly met my expectations with its predefined model mixes, quick customisation, good performance, ease of build and low budget parts. These qualities led to friends wanting it on their models and ultimately PlainFlight being refined and posted on GitHub for other hobbyists to have a go and enjoy.

While PlainFlight has been developed for small electric powered model planes it could easily be modified and used upon other small radio-controlled craft. This can be done with little effort as the code is broken down into logical modules and is well commented for those that want to understand or modify for their own purposes.

Several default pre-programmed model mixes can be chosen: Full house plane, full house plane V-tail (Talon), rudder/elevator plane, V-tail plane, or flying wing. The default code settings will compile for a 'full house plane' with control of 4 servo and 2 motor outputs, giving controls of ailerons/flap x2, elevator, rudder and throttle x2 (differential throttle also available for twin motor options).

The flight controller hardware is based upon the Seeed Studio XIAO ESP32-S3, chosen for its small size, 32bit processing power and low cost. When combined with the ever-popular MPU6050 IMU a 1KHz control loop rate is achieved which is more than ample for fixed wing aircraft. 

If you appreciate the time and effort I have invested in this project for the community, then please help me continue development with a kind gesture (Thank you):

[![Please Donate](https://github.com/plainFlight/plainFlightController/blob/main/assets/images/paypal-donate-button.png)](https://www.paypal.com/donate/?hosted_button_id=FC4SFQY3846UY)


## Specifications:
As standard PlainFlight has the following specifications:

| Feature       | Detail        |
| ------------- | ------------- |
| Model Mixes   | Plane (aileron/flaps/elevator/rudder), Plane (aileron/flaps/V-tail), Plane (rudder/elevator), Plane V-Tail (rudder/elevator), Flying Wing (elevons/rudder) |
| Flight Modes  | Pass through, gyro rate, self levelled and heading hold.  |
| Actuators     | 4 servos and 2 motors (Or any combination of the 6 with modification).  |
| Actuators Refresh | 50Hz, 100Hz, 150Hz, 250Hz, 300Hz, Oneshot125 2KHz, (Or custom with modification).|
| Motors | Direct or differential thrust. |
| Radio Protocols | Sbus. |
| Battery Monitor | 1s to 3s lipo as standard, pulses throttle on low battery, cuts throttle at minimum battery voltage.|
| Failsafe | Automatic transition to self levelled mode with user defined failsafe flight trims and cuts throttle.|
| LED | Flight mode indication.|
| Target| Seeed Studio XIAO ESP32-S3.|
| IMU| MPU6050 (GY-521 breakout board)|
| Control Loop| Stable 1KHz.|

## Hardware:

All hardware is readily avaialble from hobbyist electronics shops with the main components being:

**Seeed Studio:** [XIAO ESP32-S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) (or with slight modification XIAO ESP32-C3)

**MPU6050:** [GY-521 Breakout Board](https://www.amazon.co.uk/MPU-6050-Accelerometer-Gyroscope-Converter-Arduino/dp/B0BZXT477Z/ref=sr_1_7?crid=1PUDPKVVKYGMW&keywords=gy-521%2Bmpu6050%2Bimu&qid=1700420083&sprefix=GY-521%2Caps%2C316&sr=8-7&th=1)

The following wiring diagram details how to assemble the flight controller from component parts for a typical model. For full details on wiring and setup please see instructions/directions manual:

![Wiring diagram](https://github.com/plainFlight/plainFlightController/blob/main/assets/images/plainFlight%20Controller%20Connection%20Diagram.png)

A typical 'full house' flight controller build with 6 channels and battery monitoring:
<p align="center">
<img src="https://github.com/plainFlight/plainFlightController/blob/main/assets/images/PF_5V_Build.PNG" width="500">
</p>

## Instruction Manual:

The plainFlightController repository includes a comprehensive instruction manual that takes you through Arduino and flight controller setup for your model:

[![Instruction Manual](https://github.com/plainFlight/plainFlightController/blob/main/assets/images/Manual_9.1.1.png)](https://github.com/plainFlight/plainFlightController/blob/main/PlainFlight%20Instruction%20Manual.pdf)

## Movies:

PlainFlight controller introduction, with signature hands off knife edge:

[![Vid1](https://github.com/plainFlight/plainFlightController/blob/main/assets/images/vid2.PNG)](https://youtu.be/_j3ObBTJ5ag)

Twin differential flying wing 'A4' - Hands off take off using differential thrust and heading hold feature:

[![Vid2](https://github.com/plainFlight/plainFlightController/blob/main/assets/images/A4%20Wing.PNG)](https://youtu.be/VJwikkyEEPw)

Testing the effect of D-gain on a fixed wing model. Despite popular belief it works and from my experiences has the benefit of softening P gain 'bobbles' caused by overshoot, and removes the need for throttle attenuation of gains. NOTE: You will need a laptop sized screen for this video:

[![Vid3](https://github.com/plainFlight/plainFlightController/blob/main/assets/images/D-Gain%20Thumb.jpg)](https://youtu.be/jslFuttht5o)

## In Development

**Levelled Mode Upgrade:** Improved algorithm that uses rate mode controller for a much more locked in flight feel. This upgrade also allows the removal of levelled gains and simplifies flight tuning. Has been test flown and is working very well.

**Battery Monitor:** Improved method that provides active reduction of throttle when battery voltage is low. Working, but did not make first release.

## Planned

**Gains:** Move gains to 'EEPROM to allow gain modification via termial App or configurator App.

**IMU Board Orientation:** Allow alternative mounting options for IMU via defines.

## Community

I am open to contributions but please be aware it may take me several weeks to assess and test any submissions. Please ensure code is in the same coding style/standard and well commented. I reserve the right to decline contributions that I feel may not be in keeping with the project or standards. 

## Disclaimer:

Do not expect this software to out perform other more established flight controller projects such as ArduPilot, inav, betaFlight etc. This code shall be considered as highly experimental and is not designed or written to any safety critical, or mission critical standards. It is given/shared for free with the knowledge and understanding that this open source flight controller software is only for small hobby based electrically powered model aircraft, or other small hobby radio controlled vehicles. It is intended to be used or modified to suit your needs for small models and is NOT to be used on any manned vehicles. The author(s) shall not be held responsible or accountable for any damage, injury or loss that may be inflicted or incurred as a result of the use or missuse of this code. Use and modify at your own risk and use within accordance of your countrys laws and/or regulations. 

By using this, or any part of this software you agree to [this license agreement.](https://github.com/plainFlight/plainFlightController/blob/main/LICENSE)

...and to put it more bluntly:

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
