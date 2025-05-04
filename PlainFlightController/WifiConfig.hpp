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
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "Html.hpp"
#include "FileSystem.hpp"
#include "Config.hpp"


//using namespace Configuration;


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
    //Delimiter data type used to determine delimiter position in received web form data.
    struct Delimiter
      {
        uint32_t item[10];
        uint32_t count;
      };

    //Data type used in decoding web form data.
    struct StringHandler
    {
      Delimiter delimiter;
      String tokens[10];
      String message;
    };

    enum class WifiState : uint32_t
    {
      OFF = 0U,
      START,
      STOP,
      SERV_CLIENT
    };

    //HTML string headers received from the web application upon a web form save
    String STR_PITCH       = "GET /PITCH?";
    String STR_ROLL        = "GET /ROLL?";
    String STR_YAW         = "GET /YAW?";
    String STR_RATES       = "GET /RATES?";
    String STR_ANGLES      = "GET /ANGLE?";
    String STR_LEVEL_TRIMS = "GET /LEVEL_TRIMS?";
    String STR_SERVO_TRIMS = "GET /SERVO_TRIMS?";    
    String STR_VOLT_TRIM   = "GET /VOLT_TRIM?";
    // Set these to your desired credentials.
    static constexpr char *SSID = "PlainFlight";
    static constexpr char *PASSWORD = "12345678";
    static constexpr uint32_t HTML_DOC_BUFF_SIZE = sizeof(INDEX_HTML);

    //Methods
    void removeHeadTail(StringHandler* const str, uint32_t headerLength);
    void sendHtml(WiFiClient* const theClient);
    bool updatePidf(StringHandler* const str, PIDF::Gains* const theGains);
    bool updateRates(StringHandler* const str);
    bool updateLevelTrims(StringHandler* const str);
    bool updateServoTrims(StringHandler* const str);
    bool updateMaxAngles(StringHandler* const str);
    bool updateBatteryTrims(StringHandler* const str);

    //Variables
    FileSystem::NonVolatileData* m_webData;
    WifiState m_state = WifiState::START;
    String m_currentLine = "";
    char m_html[HTML_DOC_BUFF_SIZE] = {0};
    bool m_dataUpdated = false;
    float* m_batteryVoltage;
    float* m_pitch;
    float* m_roll;
    float* m_yaw;

    //Objects
    WiFiServer server = WiFiServer(80);
    WiFiClient client;
};