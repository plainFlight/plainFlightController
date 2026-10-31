/* 
* Original File Author: D. Gamble (Github: Cyberslug)
*
* Copyright (c) 2026 P.Cook (alias 'plainFlight')
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
* @file   SerialMonitor.cpp
* @brief  This class implements DEBUG outputs via the serial console port.
*/

#include "SerialMonitor.hpp"
#include "FlightControl.hpp"


/**
* @brief  Called once at the end of FlightControl::begin().
* @param  fc  The flight controller being monitored.
*/
void
SerialMonitor::begin(const FlightControl& fc)
{
  if constexpr(SOURCE == MonitorSource::BATTERY_MONITOR)
  {
    Serial.print("Cells detected: ");
    Serial.println(fc.getBatteryMonitor().getNumberCells());
  }
  if constexpr (SOURCE == MonitorSource::RX)
  {
    m_rxWindowStartMs = millis();
  }
}


/**
* @brief  Called at the end of every loop.
* @param  fc  The flight controller being monitored.
*/
void
SerialMonitor::update(const FlightControl& fc)
{
  if constexpr (SOURCE == MonitorSource::LOOP_RATE)
  {
    printLoopRate(fc);
  }
  else if constexpr (SOURCE == MonitorSource::RC_DATA)
  {
    printRcData(fc);
  }
  else if constexpr (SOURCE == MonitorSource::BATTERY_MONITOR)
  {
    printBattery(fc);
  }
  else if constexpr (SOURCE == MonitorSource::MADGWICK)
  {
    printMadgwick(fc);
  }
  else if constexpr (SOURCE == MonitorSource::IMU)
  {
    printImu(fc);
  }
  else if constexpr (SOURCE == MonitorSource::OUTPUTS)
  {
    printOutputs(fc);
  }
  else if constexpr (SOURCE == MonitorSource::GPS)
  {
    printGps(fc);
  }
  else if constexpr (SOURCE == MonitorSource::RX)
  {
    printRx(fc);
  }
}


/**
* @brief   Rate limits the console output.
* @return  True once every PRINT_PERIOD_MS, and only while the serial port is connected.
*/
bool
SerialMonitor::printDue()
{
  const uint32_t now = millis();

  // Signed difference so the comparison still works when millis() wraps.
  if ((static_cast<int32_t>(now - m_nextPrintMs) >= 0) && Serial)
  {
    m_nextPrintMs = now + PRINT_PERIOD_MS;
    return true;
  }

  return false;
}


/**
* @brief  Loop period, cycle time and average cycle time, in seconds.
* @param  fc  The flight controller being monitored.
* @note   The average is updated every loop; only the printing is rate limited.
*/
void
SerialMonitor::printLoopRate(const FlightControl& fc)
{
  const float cycleTimeUs = static_cast<float>(fc.getCycleTimeUs());
  m_avCycleTimeUs = ((m_avCycleTimeUs * 999.0f) + cycleTimeUs) / 1000.0f;

  if (printDue())
  {
    Serial.print(static_cast<float>(fc.getLoopPeriodUs()) / 1000000.0f, 6);
    Serial.print("\t");
    Serial.print(cycleTimeUs / 1000000.0f, 6);
    Serial.print("\t");
    Serial.println(m_avCycleTimeUs / 1000000.0f, 6);
  }
}


/**
* @brief    Prints DemandProcessor data to console for debugging purposes.
* @param  fc  The flight controller being monitored.
*/
void
SerialMonitor::printRcData(const FlightControl& fc)
{
  if (printDue())
  {
    const  DemandProcessor::Demands* const demand = fc.getRc().getDemands();
    Serial.printf("pitch: %5d roll: %5d yaw: %5d throttle: %5d flaps: %5d %8s state: %2d ",
      demand->pitch, demand->roll, demand->yaw, demand->throttle, demand->flaps, 
      demand->armed ? "ARMED": "DISARMED", static_cast<uint8_t>(fc.getFlightState()));

    if constexpr(Config::USE_HEADING_HOLD)
    {
      Serial.printf(" HeadingHold: %3s", demand->headingHold ? "On" : "Off");
    }

    if constexpr(Config::USE_PROP_HANG_MODE)
    {
      Serial.printf(" PropHang: %3s", demand->propHang ? "On" : "Off");
    } 

    Serial.println();
  }
}


/**
* @brief    Prints battery debug information to the console
* @param  fc  The flight controller being monitored.
*/ 
void
SerialMonitor::printBattery(const FlightControl& fc)
{
  if (printDue())
  {
    const BatteryMonitor& battery = fc.getBatteryMonitor();

    Serial.print("ADC: ");
    Serial.print(battery.getRawAdc());
    Serial.print(", Cells: ");
    Serial.print(battery.getNumberCells());
    Serial.print(", Scaler:");
    Serial.print(fc.getConfig().getBatteryScaler(), 5);
    Serial.print(", volts:");
    Serial.println(battery.getVoltage());
  }
}


/**
* @brief    Roll and pitch from the Madgwick filter (yaw too in prop hang builds)
* @param  fc  The flight controller being monitored.
*/ 
void
SerialMonitor::printMadgwick(const FlightControl& fc)
{
  if (printDue())
  {
    const IMU::ImuData* const imu = fc.getImuData();

    Serial.printf("Roll: %7.2f, Pitch: %7.2f", imu->roll, imu->pitch);

    if constexpr(Config::USE_PROP_HANG_MODE)
    {
      Serial.printf(", Yaw: %7.2f", imu->yaw);
    }
    Serial.println();
  }
}


/**
* @brief  Scaled gyro (deg/s) and accel (g) from the IMU sensor; reports a read error once.
* @param  fc  The flight controller being monitored.
*/
void
SerialMonitor::printImu(const FlightControl& fc)
{
  const bool imuOk = fc.getImu().isOk();

  if (!imuOk && m_lastImuOk && Serial)
  {
     Serial.println("IMU read error!");
  }
  m_lastImuOk = imuOk;

  if (printDue())
  {
    const Mpu6050::MpuData sensor = fc.getImuData()->mpu6050;

    Serial.printf("gx: %7.2f gy: %7.2f gz: %7.2f ax: %6.2f ay: %6.2f az: %6.2f\n",
      sensor.gyro_X, sensor.gyro_Y, sensor.gyro_Z,
      sensor.accel_X, sensor.accel_Y, sensor.accel_Z);
  }
}


/**
* @brief  Output timer ticks on one line: servos (S), motors (M), pass-through (P).
* @param  fc  The flight controller being monitored.
*/
void
SerialMonitor::printOutputs(const FlightControl& fc)
{
  if (printDue())
  {
    const ModelBase* const model  = fc.getModel();
    const uint8_t servos          = model->getNumberServos();
    const uint8_t motors          = model->getNumberMotors();
    const uint8_t passThrough     = model->getNumberPassThrough();
    uint8_t index = 0U;

    for (uint8_t i = 0U; i < servos; i++)
    {
      Serial.printf("S%u: %5lu  ", i, static_cast<unsigned long>(model->getOutputTicks(index++)));
    }
    for (uint8_t i = 0U; i < motors; i++)
    {
      Serial.printf("M%u: %5lu  ", i, static_cast<unsigned long>(model->getOutputTicks(index++)));
    }
    for (uint8_t i = 0U; i < passThrough; i++)
    {
      Serial.printf("P%u: %5lu  ", i, static_cast<unsigned long>(model->getOutputTicks(index++)));
    }
    Serial.println();
  }
}


/**
* @brief  Output a summary of the latest Gps data
* @param  fc  The flight controller being monitored.
*/
void
SerialMonitor::printGps(const FlightControl& fc)
{
  const GnssData& gnss = fc.getLastGnss();

  if ((gnss.second != m_lastGnssSecond) || (gnss.millisecond != m_lastGnssMs))
  {
    m_lastGnssSecond = gnss.second;
    m_lastGnssMs = gnss.millisecond;

    if (printDue())
    {
      Serial.printf("%02d:%02d:%02d %02d Lat %07d, Long %07d\n", gnss.hour, gnss.minute, 
        gnss.second, gnss.satellites, gnss.latitude, gnss.longitude);
    }
  }
}


/**
* @brief  Receiver frame rate, normalised channels, failsafe and lost comms.
* @param  fc  The flight controller being monitored.
* @note   Frames are counted every loop; the rate is the count over the print window.
*/
void
SerialMonitor::printRx(const FlightControl& fc)
{
  const DemandProcessor& rc = fc.getRc();
  const RxBase* const receiver = rc.getReceiver();
  const bool lostComms = (nullptr != receiver) && receiver->hasLostCommunications();

  if (rc.hasNewRxPacket())
  {
    m_rxFrames++;
  }

  const uint32_t now = millis();
  const uint32_t windowMs = now - m_rxWindowStartMs;
  if (windowMs >= RX_RATE_WINDOW_MS)
  {
    m_rxHz = (m_rxFrames * 1000U) / windowMs;
    m_rxFrames = 0U;
    m_rxWindowStartMs = now;
  }

  if (printDue())
  {
    const RxBase::RxPacket* const packet = rc.getNormalisedRcData();

    Serial.printf("Hz: %4lu ", static_cast<unsigned long>(m_rxHz));
    for (uint32_t i = 0U; i < RxBase::MAX_RX_CHANNELS; i++)
    {
      Serial.printf(" %5ld", static_cast<long>(packet->ch[i]));
    }
    Serial.printf("  failsafe: %u lostComms: %u\n", packet->failsafe, lostComms);
  }
}
