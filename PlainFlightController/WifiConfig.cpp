/* 
* Copyright (c) 2025, 2026 P.Cook (alias 'plainFlight')
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
* @file   WifiConfig.cpp
* @brief  This class contains methods that controls the wifi and HTML web page.
*/

#include "WifiConfig.hpp"
#include "Config.hpp"
#include "InternalConfig.hpp"
#include "Monitor.hpp"


/**
* @brief    Constructore.
* @param    Parameters to be displayed/updated by the WiFi configurator
* @note     TODO - this can be done better in the hpp.
*/
WifiConfig::WifiConfig(FileSystem::NonVolatileData * const theData, float* const batteryVoltage, float* const pitch, float* const roll, float* const yaw)
{
  m_webData = theData;
  m_batteryVoltage = batteryVoltage;
  m_pitch = pitch;
  m_roll = roll;
  m_yaw = yaw;
}


/**
* @brief  Wifi state machine enabled by high throttle stick position when disarmed.
*/
void 
WifiConfig::doWiFiStateMachine()
{
  switch (m_state)
  {
    case WifiState::OFF:
      break;

    case WifiState::START:
    {
      Monitor::event(MonitorSource::CONFIGURATOR, "start_wifi");
      const bool ok = startWifiConfigurator();
      m_state = (ok) ? WifiState::SERV_CLIENT : WifiState::OFF;
      break;
    }

    default:
    case WifiState::STOP:
      Monitor::event(MonitorSource::CONFIGURATOR, "stop_wifi");
      stopWifiConfigurator();
      m_state = WifiState::OFF;
      break;
    
    case WifiState::SERV_CLIENT:
    {
      //serviceWifiConfigurator();
      dnsServer.processNextRequest();
      server.handleClient();
      break;  
    } 
  }
}


/**
* @brief  Initiates a wifi access point and server.
* @return True if wifi started.
*/
bool 
WifiConfig::startWifiConfigurator() 
{
  Monitor::event(MonitorSource::CONFIGURATOR, "Configuring access point...");

  IPAddress m_localIP(192,168,4,1);
  IPAddress m_gateway(192,168,4,1);
  IPAddress m_subnet(255,255,255,0);

  WiFi.softAPConfig(m_localIP, m_gateway, m_subnet);

  if (!WiFi.softAP(SSID, PASSWORD))
  {
    Monitor::event(MonitorSource::CONFIGURATOR, "Soft AP failed.");
    return false;
  }

  m_localIP = WiFi.softAPIP();
  //Using dns to create a captive portal in an effort to automatically open a webpage.
  dnsServer.start(53, "*", m_localIP);

  //Pages
  server.on(STR_PAGE_ROOT, HTTP_GET, [this](){handleRoot();});
  server.on(STR_PAGE_MAIN, HTTP_GET, [this](){sendMain();});
  
  //Forms
  server.on(STR_FORM_PITCH, HTTP_POST, [this](){handlePitchGains();});
  server.on(STR_FORM_ROLL, HTTP_POST, [this](){handleRollGains();});
  server.on(STR_FORM_YAW, HTTP_POST, [this](){handleYawGains();});
  server.on(STR_FORM_RATES, HTTP_POST, [this](){handleDegreeRates();});
  server.on(STR_FORM_ANGLE, HTTP_POST, [this](){handleMaxLevelAngles();});
  server.on(STR_FORM_LEVEL_TRIMS, HTTP_POST, [this](){handleLevelTrims();});
  server.on(STR_FORM_SERVO_TRIMS, HTTP_POST, [this](){handleServoTrims();});
  server.on(STR_FORM_VOLT_TRIM, HTTP_POST, [this](){handleBatteryTrim();});

  //Captive portal detection
  server.on(STR_PORTAL_ROOT_1, HTTP_GET, [this](){handleRoot();});
  server.on(STR_PORTAL_ROOT_2, HTTP_GET, [this](){handleRoot();});
  server.on(STR_PORTAL_ROOT_3, HTTP_GET, [this](){handleRoot();});
  server.on(STR_PORTAL_ROOT_4, HTTP_GET, [this](){handleRoot();});
  server.on(STR_PORTAL_ROOT_5, HTTP_GET, [this](){handleRoot();});
  server.onNotFound([this](){handleNotFound();});

  //Handle live updates
  server.on(STR_LIVE_UPDATE_MODEL, HTTP_GET, [this](){handleModelUpdates();});

  server.begin();
  return true;
}


/**
* @brief  Disconnects and turns off wifi.
* @note   Wifi is always off when the model is armed.
*/
void 
WifiConfig::stopWifiConfigurator() 
{
  Monitor::event(MonitorSource::CONFIGURATOR, "Client Disconnected.");
  WiFi.softAPdisconnect();
  WiFi.mode(WIFI_OFF);
  Monitor::event(MonitorSource::CONFIGURATOR, "Wifi stopped");
}


/**
* @brief  Informs caller when new data has been captured.
* @return true when a value has been updated.
*/
bool 
WifiConfig::hasUpdatedData()
{
  const bool updated = m_dataUpdated;
  m_dataUpdated = false;
  return updated;
}


/**
* @brief    Send the main HTML page.
*/
void
WifiConfig::sendMain()
{
  int32_t degreesPerSec = 0;

  if constexpr(Config::GYRO_RATE == GyroRate::IS_250_DEGS_SECOND)
  {
    degreesPerSec = 250;
  }
  else
  {
    if constexpr(Config::GYRO_RATE == GyroRate::IS_500_DEGS_SECOND)
    {
      degreesPerSec = 500;
    }
  }

  const int32_t n = snprintf (m_html, HTML_DOC_BUFF_SIZE, INDEX_HTML, 
                          InternalConfig::SOFTWARE_VERSION,
                          m_webData->gains.pitch.p, m_webData->gains.pitch.i, (m_webData->gains.pitch.d/10), m_webData->gains.pitch.ff,
                          m_webData->gains.roll.p, m_webData->gains.roll.i, (m_webData->gains.roll.d/10), m_webData->gains.roll.ff,
                          m_webData->gains.yaw.p, m_webData->gains.yaw.i, (m_webData->gains.yaw.d/10), m_webData->gains.yaw.ff,
                          (m_webData->rates.pitch/100), degreesPerSec, (m_webData->rates.roll/100), degreesPerSec, (m_webData->rates.yaw/100), degreesPerSec, 
                          (m_webData->maxAngle.pitch/100), (m_webData->maxAngle.roll/100), 
                          *m_pitch, *m_roll, *m_yaw, m_webData->levelTrim.pitch, m_webData->levelTrim.roll, m_webData->levelTrim.yaw,
                          m_webData->servoTrim.servo1, m_webData->servoTrim.servo2, m_webData->servoTrim.servo3, m_webData->servoTrim.servo4,
                          *m_batteryVoltage, m_webData->batteryScaler);

  if (HTML_DOC_BUFF_SIZE < (n+1))
  {
    Serial.println("snprintf buff size error !");
  }
  else
  {
    server.send(200, STR_TEXT_HTML, m_html);
  }
}
    

/**
* @brief    Handle root.
*/
void
WifiConfig::handleRoot()
{
  Monitor::event(MonitorSource::CONFIGURATOR, "handle root");

  server.sendHeader(STR_LOCATION, STR_ESP_MAIN_URL, true);
  server.send(302, STR_TEXT_PLAIN, "");
}


/**
* @brief    Handle web requests that are not known.
*/
void
WifiConfig::handleNotFound()
{
  Monitor::event(MonitorSource::CONFIGURATOR, "handle not found");

  server.sendHeader(STR_LOCATION, STR_ESP_MAIN_URL, true);
  server.send(302, STR_TEXT_PLAIN, "");
}


/**
* @brief    Common decoding method for all PIDF gains.
* @param    Pointer to gains structure to populate.
*/
void
WifiConfig::updateGains(PIDF::Gains* const theGains)
{
  const uint32_t P = server.arg(ARG_P).toInt();
  const uint32_t I = server.arg(ARG_I).toInt();
  const uint32_t D = server.arg(ARG_D).toInt() * 10;
  const uint32_t F = server.arg(ARG_F).toInt();

  Monitor::eventf(MonitorSource::CONFIGURATOR, "Gains updated: P:%lu I:%lu D:%lu F:%lu", P, I, D, F);

  theGains->p = P;
  theGains->i = I;
  theGains->d = D;
  theGains->ff = F;
}


/**
* @brief    Decode the pitch gains.
*/
void
WifiConfig::handlePitchGains()
{
  updateGains(&m_webData->gains.pitch);
  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Decode the roll gains.
*/
void
WifiConfig::handleRollGains()
{
  updateGains(&m_webData->gains.roll);
  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Decode the yaw gains.
*/
void
WifiConfig::handleYawGains()
{
  updateGains(&m_webData->gains.yaw);
  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Decode the degrees pers second rates
*/
void
WifiConfig::handleDegreeRates()
{
  const uint32_t pitch = static_cast<uint32_t>(server.arg(ARG_PITCH).toInt() * 100);
  const uint32_t roll = static_cast<uint32_t>(server.arg(ARG_ROLL).toInt() * 100);
  const uint32_t yaw = static_cast<uint32_t>(server.arg(ARG_YAW).toInt() * 100);

  Monitor::eventf(MonitorSource::CONFIGURATOR, "Degree rates updated: Pitch:%lu Roll:%lu Yaw:%lu", pitch, roll, yaw);

  m_webData->rates.pitch = pitch;
  m_webData->rates.roll = roll;
  m_webData->rates.yaw = yaw;

  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Decode the max angle trims
*/
void
WifiConfig::handleMaxLevelAngles()
{
  const uint32_t pitch = static_cast<uint32_t>(server.arg(ARG_PITCH).toInt() * 100);
  const uint32_t roll = static_cast<uint32_t>(server.arg(ARG_ROLL).toInt() * 100);

  Monitor::eventf(MonitorSource::CONFIGURATOR, "Max level angles updated: Pitch:%lu Roll:%lu", pitch, roll);

  m_webData->maxAngle.pitch = pitch;
  m_webData->maxAngle.roll = roll;

  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Decode the new level trims
*/
void
WifiConfig::handleLevelTrims()
{
  const float pitch = server.arg(ARG_PITCH).toFloat();
  const float roll = server.arg(ARG_ROLL).toFloat();
  const float yaw = server.arg(ARG_YAW).toFloat();

  Monitor::eventf(MonitorSource::CONFIGURATOR, "Level trims updated: Pitch:%.2f Roll:%.2f Yaw:%.2f", pitch, roll, yaw);

  m_webData->levelTrim.pitch = pitch;
  m_webData->levelTrim.roll = roll;
  m_webData->levelTrim.yaw = yaw;     //Note: This is used to trim level when in prophang mode

  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Decode the new servo trims
*/
void
WifiConfig::handleServoTrims()
{
  const int32_t servo1 = server.arg(ARG_SERVO1).toInt();
  const int32_t servo2 = server.arg(ARG_SERVO2).toInt();
  const int32_t servo3 = server.arg(ARG_SERVO3).toInt();
  const int32_t servo4 = server.arg(ARG_SERVO4).toInt();

  Monitor::eventf(MonitorSource::CONFIGURATOR, "Servo trims updated: servo1:%ld servo2:%ld servo3:%ld servo4:%ld",
                  servo1, servo2, servo3, servo4);

  m_webData->servoTrim.servo1 = servo1;
  m_webData->servoTrim.servo2 = servo2;
  m_webData->servoTrim.servo3 = servo3;
  m_webData->servoTrim.servo4 = servo4;

  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Decode the new battery scaler
*/
void
WifiConfig::handleBatteryTrim()
{
  const float voltCalibration = server.arg(ARG_VOLTS).toFloat();

  Monitor::eventf(MonitorSource::CONFIGURATOR, "Voltage calibration updated: Cal:%.5f", voltCalibration);

  m_webData->batteryScaler = voltCalibration;

  server.sendHeader(STR_LOCATION, STR_PAGE_MAIN);
  server.send(303);
  m_dataUpdated = true;
}


/**
* @brief    Process and send live updates.
*/
void
WifiConfig::handleModelUpdates()
{
  constexpr uint8_t BUFF_SIZE = 96U;
  char json[BUFF_SIZE] = {0};

  const int32_t n = snprintf(
            json, sizeof(json),
            "{\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f,\"volts\":%.2f}",
            *m_pitch, *m_roll, *m_yaw, *m_batteryVoltage);

  Monitor::eventf(MonitorSource::CONFIGURATOR, "Model update: %s", json);

  if (BUFF_SIZE < (n+1))
  {
    Serial.println("snprintf buff size error !");
  }
  else
  {
    server.send(200, STR_APP_JSON, json);
  }
}
