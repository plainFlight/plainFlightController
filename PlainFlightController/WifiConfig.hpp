/* 
* Copyright (c) 2025 P.Cook (alias 'plainFlight')
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
* @file   WifiConfig.hpp
* @brief  This class contains methods that controls the wifi and HTML web page.
*/

#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "Html.hpp"
#include "FileSystem.hpp"
#include "Config.hpp"



/**
* @class  WifiConfig
* @note   Inherits Html class.
*/
class WifiConfig : public Html
{
  public:
    WifiConfig(FileSystem::NonVolatileData* const theData, float* const batteryVoltage, float* const pitch, float* const roll, float* const yaw);
    void doWiFiStateMachine();
    bool startWifiConfigurator();
    void stopWifiConfigurator();
    void serviceWifiConfigurator();
    bool hasUpdatedData();

  private:

    enum class WifiState : uint32_t
    {
      OFF = 0U,
      START,
      STOP,
      SERV_CLIENT
    };

    // Set these to your desired credentials.
    static constexpr char SSID[] = "PlainFlight";
    static constexpr char PASSWORD[] = "12345678";
    static constexpr uint32_t HTML_VARIABLES_SIZE = 200U;
    static constexpr uint32_t HTML_DOC_BUFF_SIZE = sizeof(INDEX_HTML) + HTML_VARIABLES_SIZE;
    //Constants to scale display data into sensible values i.e. display 10's or 100's not 1000's or 10,000's.
    static constexpr int32_t SCALE_D_GAIN     = 10;
    static constexpr int32_t SCALE_RATES      = 100;
    static constexpr int32_t SCALE_MAX_ANGLE  = 100;
    //Captive portal strings
    static constexpr char STR_PORTAL_ROOT_1[] = "/generate_204";
    static constexpr char STR_PORTAL_ROOT_2[] = "/hotspot-detect.html";
    static constexpr char STR_PORTAL_ROOT_3[] = "/connecttest.txt";
    static constexpr char STR_PORTAL_ROOT_4[] = "/ncsi.txt";
    static constexpr char STR_PORTAL_ROOT_5[] = "/fwlink";
    //Pages
    static constexpr char STR_PAGE_ROOT[] = "/";
    static constexpr char STR_PAGE_MAIN[] = "/main";
    //Forms
    static constexpr char STR_FORM_PITCH[] = "/PITCH";
    static constexpr char STR_FORM_ROLL[] = "/ROLL";
    static constexpr char STR_FORM_YAW[] = "/YAW";
    static constexpr char STR_FORM_RATES[] = "/RATES";
    static constexpr char STR_FORM_ANGLE[] = "/ANGLE";
    static constexpr char STR_FORM_LEVEL_TRIMS[] = "/LEVEL_TRIMS";
    static constexpr char STR_FORM_SERVO_TRIMS[] = "/SERVO_TRIMS";
    static constexpr char STR_FORM_VOLT_TRIM[] = "/VOLT_TRIM";
    //Handle live updates
    static constexpr char STR_LIVE_UPDATE_MODEL[] = "/MODEL";
    //Strings
    static constexpr char STR_TEXT_PLAIN[] = "text/plain";
    static constexpr char STR_TEXT_HTML[] = "text/html";
    static constexpr char STR_APP_JSON[] = "application/json";
    static constexpr char STR_LOCATION[] = "Location";
    static constexpr char STR_ESP_MAIN_URL[] = "http://192.168.4.1/main";
    //Received argument strings
    static constexpr char ARG_P[]     = "P";
    static constexpr char ARG_I[]     = "I";
    static constexpr char ARG_D[]     = "D";
    static constexpr char ARG_DFF[]   = "DFF";
    static constexpr char ARG_F[]     = "F";
    static constexpr char ARG_VOLTS[] = "volts";
    static constexpr char ARG_PITCH[] = "pitch";
    static constexpr char ARG_ROLL[]  = "roll";
    static constexpr char ARG_YAW[]   = "yaw";
    static constexpr char ARG_SERVO1[] = "Servo1";
    static constexpr char ARG_SERVO2[] = "Servo2";
    static constexpr char ARG_SERVO3[] = "Servo3";
    static constexpr char ARG_SERVO4[] = "Servo4";

    //Methods
    void updateGains(PIDF::Gains* const theGains);
    void sendMain();
    void handleRoot();
    void handleNotFound();
    //Forms methods
    void handlePitchGains();
    void handleRollGains();
    void handleYawGains();
    void handleDegreeRates();
    void handleMaxLevelAngles();
    void handleLevelTrims();
    void handleServoTrims();
    void handleBatteryTrim();
    //Methods for handling live updates
    void handleModelUpdates();

    //Variables
    FileSystem::NonVolatileData* m_webData;
    WifiState m_state = WifiState::START;
    char m_html[HTML_DOC_BUFF_SIZE] = {0};
    bool m_dataUpdated = false;
    float* m_batteryVoltage;
    float* m_pitch;
    float* m_roll;
    float* m_yaw;

    //Objects
    DNSServer dnsServer;
    WebServer server = WebServer(80);
};