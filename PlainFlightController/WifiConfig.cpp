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
* @file   WifiConfig.cpp
* @brief  This class contains methods that controls the wifi and HTML web page.
*/

#include "WifiConfig.hpp"
#include "Config.hpp"


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
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("wifi_off");}
      break;

    case WifiState::START:
    {
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("start_wifi");}
      const bool ok = startWifiConfigurator();
      m_state = (ok) ? WifiState::SERV_CLIENT : WifiState::OFF;
      break;
    }

    default:
    case WifiState::STOP:
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("stop_wifi");}
      stopWifiConfigurator();
      m_state = WifiState::OFF;
      break;
    
    case WifiState::SERV_CLIENT:
    {
      serviceWifiConfigurator();
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
  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Configuring access point...");}

  IPAddress m_localIP(192,168,4,1);
  IPAddress m_gateway(192,168,1,0);
  IPAddress m_subnet(255,255,255,0);

  WiFi.softAPConfig(m_localIP, m_gateway, m_subnet);

  // You can remove the password parameter if you want the AP to be open.
  // a valid password must have more than 7 characters
  if (!WiFi.softAP(SSID, PASSWORD)) 
  {
    if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Soft AP creation failed.");}
    return false;
  }
  else
  {
    IPAddress myIP = WiFi.softAPIP();
    if constexpr(Config::DEBUG_CONFIGURATOR)
    {
      Serial.print("AP IP address: ");
      Serial.println(myIP);
    }
    
    server.begin();
    if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Server started");}    
    return true;
  }
}


/**
* @brief  Disconnects and turns off wifi.
* @note   Wifi is always off when the model is armed.
*/
void 
WifiConfig::stopWifiConfigurator() 
{
  client.stop();
  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Client Disconnected.");}
  WiFi.softAPdisconnect();
  WiFi.mode(WIFI_OFF);
  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Wifi stopped");}
}


/**
* @brief  Main wifi routine that handles client connection and HTML traffic.
* @note   Probably better ways of doing this but wanted to avoid unreliable 3rd party libraries.
*/
void 
WifiConfig::serviceWifiConfigurator()//FileSystem::NonVolatileData * const fileData) 
{
  if (!client)
  {
    client = server.accept();   // listen for incoming clients
    m_currentLine = "";
  }
  else 
  { 
    // if you get a client,
    if (client.connected()) 
    {           
      // loop while the client's connected
      if (client.available()) 
      {         
        // if there's bytes to read from the client        
        char c = client.read();             // read a byte, then
  
        if (c == '\n')
        {                    
          // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (m_currentLine.length() == 0) 
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Send HTML doc.");}
            sendHtml(&client);
            // The HTTP response ends with another blank line:
            client.println();
            client.stop();
            if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Client Disconnected.");}
          }          
          else 
          {     
            StringHandler strData = {0}; 
            // if you got a newline, then clear currentLine:
            if (m_currentLine.startsWith(STR_PITCH))
            {       
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received pitch PID.");}
              removeHeadTail(&strData, STR_PITCH.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}

              if (updatePidf(&strData, &m_webData->gains.pitch))
              {
                m_dataUpdated = true;
              }
            }
            else if (m_currentLine.startsWith(STR_ROLL))
            {
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received roll PID.");}
              removeHeadTail(&strData, STR_ROLL.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}

              if (updatePidf(&strData, &m_webData->gains.roll))
              {
                m_dataUpdated = true;
              }
            }
            else if (m_currentLine.startsWith(STR_YAW))
            {
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received Yaw PID.");}
              removeHeadTail(&strData, STR_YAW.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}

              if (updatePidf(&strData, &m_webData->gains.yaw))
              {
                m_dataUpdated = true;
              }
            }
            else if (m_currentLine.startsWith(STR_RATES))
            {
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received Rates.");}
              removeHeadTail(&strData, STR_RATES.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}

              if (updateRates(&strData))
              {
                m_dataUpdated = true;
              }
            }
            else if (m_currentLine.startsWith(STR_ANGLES))
            {
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received max angles PID.");}
              removeHeadTail(&strData, STR_ANGLES.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}
 
              if (updateMaxAngles(&strData))
              {
                m_dataUpdated = true;
              }
            }
            else if (m_currentLine.startsWith(STR_LEVEL_TRIMS))
            {
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received level trims.");}
              removeHeadTail(&strData, STR_LEVEL_TRIMS.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}

              if (updateLevelTrims(&strData))
              {
                m_dataUpdated = true;
              }
            } 
            else if (m_currentLine.startsWith(STR_SERVO_TRIMS))
            {
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received level trims.");}
              removeHeadTail(&strData, STR_SERVO_TRIMS.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}

              if (updateServoTrims(&strData))
              {
                m_dataUpdated = true;
              }
            } 
            else if (m_currentLine.startsWith(STR_VOLT_TRIM))
            {
              strData.message = m_currentLine;
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Received battery.");}
              removeHeadTail(&strData, STR_VOLT_TRIM.length());
              if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(strData.message);}

              if (updateBatteryTrims(&strData))
              {
                m_dataUpdated = true;
              }
            }       
            
            m_currentLine = "";
          }          
        } 
        else if (c != '\r') 
        {  // if you got anything else but a carriage return character,
          m_currentLine += c;      // add it to the end of the currentLine
        }        
      }
    }
  }
}


/**
* @brief    Remove HTML fomatting from client string to get data that we are interested in.
* @param    str The received string to trim, header length to remove.
* @param    headerLength The length of header.
*/
void 
WifiConfig::removeHeadTail(StringHandler* const str, uint32_t headerLength)
{
  int32_t idx;

  str->message.remove(0, headerLength);
  //Find " HTTP/1.1"
  for (uint32_t i=0; i<str->message.length(); i++)
  {
    if(str->message.charAt(i) == ' ')
    {
      idx = i;
      break;
    }
  }
  //Remove " HTTP/1.1"
  str->message.remove(idx, str->message.length());
  //Find all '&' characters
  str->delimiter.count = 0U;

  for (int32_t i=0; i<str->message.length(); i++)
  {
    if(str->message.charAt(i) == '&')
    {
      str->delimiter.item[str->delimiter.count] = i;
      str->delimiter.count++;
    }    
  }

  //Add string length to indicate EOM as a delimeter
  str->delimiter.item[str->delimiter.count] = str->message.length();
}


/**
* @brief  Sends HTML webpage to the client.
* @param  theClient Connected client to send data to, system data to export to client.
*/
void 
WifiConfig::sendHtml(WiFiClient* const theClient)
{  
  int32_t degreesPerSec = 0;

  if constexpr(Config::USE_250_DEGS_SECOND)
  {
    degreesPerSec = 250;
  }

  if constexpr(Config::USE_500_DEGS_SECOND)
  {
    degreesPerSec = 500;
  }

  const int32_t n = snprintf (m_html, HTML_DOC_BUFF_SIZE, INDEX_HTML, 
                          Config::SOFTWARE_VERSION,
                          m_webData->gains.pitch.p, m_webData->gains.pitch.i, m_webData->gains.pitch.d, m_webData->gains.pitch.ff,
                          m_webData->gains.roll.p, m_webData->gains.roll.i, m_webData->gains.roll.d, m_webData->gains.roll.ff,
                          m_webData->gains.yaw.p, m_webData->gains.yaw.i, m_webData->gains.yaw.d, m_webData->gains.yaw.ff,
                          (m_webData->rates.pitch/100), degreesPerSec, (m_webData->rates.roll/100), degreesPerSec, (m_webData->rates.yaw/100), degreesPerSec, 
                          (m_webData->maxAngle.pitch/100), (m_webData->maxAngle.roll/100), 
                          *m_pitch, *m_roll, *m_yaw, m_webData->levelTrim.pitch, m_webData->levelTrim.roll, m_webData->levelTrim.yaw, 
                          m_webData->servoTrim.servo1, m_webData->servoTrim.servo2, m_webData->servoTrim.servo3, m_webData->servoTrim.servo4,
                          *m_batteryVoltage, m_webData->batteryScaler);

  theClient->print(m_html);
}


/**
* @brief  Updates PIDF parameters.
* @param  str String containing data which to decode.
* @param  theGains  Pitch, roll or yaw gains.
* @return true when a value has been updated.
*/
bool 
WifiConfig::updatePidf(StringHandler* const str, PIDF::Gains* const theGains)
{
  bool updated = false;

  for(int32_t i=0, j=0; i<=str->delimiter.count; i++)
  {
    str->tokens[i] = str->message.substring(j, str->delimiter.item[i]);
     j = str->delimiter.item[i] + 1;

    if(str->tokens[i].startsWith("P="))
    {    
      str->tokens[i].remove(0,2);
      theGains->p = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(theGains->p);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("I="))
    {    
      str->tokens[i].remove(0,2);
      theGains->i = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(theGains->i);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("D="))
    {    
      str->tokens[i].remove(0,2);
      theGains->d = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(theGains->d);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("F="))
    {    
      str->tokens[i].remove(0,2);
      theGains->ff = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(theGains->ff);}
      updated = true;
    }
    else
    {
      ;
    }
  }

  return updated;
}


/**
* @brief  Updates rate parameters.
* @param  str String containing data which to decode.
* @return true when a value has been updated.
*/
bool 
WifiConfig::updateRates(StringHandler* const str)
{
  bool updated = true;

  for(int32_t i=0, j=0; i<=str->delimiter.count; i++)
  {
    str->tokens[i] = str->message.substring(j, str->delimiter.item[i]);
     j = str->delimiter.item[i] + 1;

    if(str->tokens[i].startsWith("pitch="))
    {    
      str->tokens[i].remove(0,6);
      m_webData->rates.pitch = str->tokens[i].toInt() * 100;
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->rates.pitch);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("roll="))
    {    
      str->tokens[i].remove(0,5);
      m_webData->rates.roll = str->tokens[i].toInt() * 100;
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->rates.roll);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("yaw="))
    {    
      str->tokens[i].remove(0,4);
      m_webData->rates.yaw = str->tokens[i].toInt() * 100;
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->rates.yaw);}
      updated = true;
    }
    else
    {
      ;
    }
  }

  return updated;
}


/**
* @brief  Updates level mode trims parameters.
* @param  str String containing data which to decode.
* @return true when a value has been updated.
*/
bool 
WifiConfig::updateLevelTrims(StringHandler* const str)
{
  bool updated = false;

  for(int32_t i=0, j=0; i<=str->delimiter.count; i++)
  {
    str->tokens[i] = str->message.substring(j, str->delimiter.item[i]);
     j = str->delimiter.item[i] + 1;

    if(str->tokens[i].startsWith("pitch="))
    {    
      str->tokens[i].remove(0,6);
      m_webData->levelTrim.pitch = str->tokens[i].toFloat();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->levelTrim.pitch);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("roll="))
    {    
      str->tokens[i].remove(0,5);
      m_webData->levelTrim.roll = str->tokens[i].toFloat();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->levelTrim.roll);}
      updated = true;
    }
    else
    {
      if(str->tokens[i].startsWith("yaw="))
      {    
        str->tokens[i].remove(0,4);
        m_webData->levelTrim.yaw = str->tokens[i].toFloat();
        if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->levelTrim.yaw);}
        updated = true;
      }
    }
  }

  return updated;
}


/**
* @brief  Updates max bank angles for levelled mode parameters.
* @param  str String containing data which to decode.
* @return true when a value has been updated.
*/
bool 
WifiConfig::updateMaxAngles(StringHandler* const str)
{
  bool updated = false;

  for(int32_t i=0, j=0; i<=str->delimiter.count; i++)
  {
    str->tokens[i] = str->message.substring(j, str->delimiter.item[i]);
     j = str->delimiter.item[i] + 1;

    if(str->tokens[i].startsWith("pitch="))
    {    
      str->tokens[i].remove(0,6);
      m_webData->maxAngle.pitch = str->tokens[i].toInt() * 100;
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->maxAngle.pitch);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("roll="))
    {    
      str->tokens[i].remove(0,5);
      m_webData->maxAngle.roll = str->tokens[i].toInt() * 100;
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->maxAngle.roll);}
      updated = true;
    }
    else
    {
      ;
    }
  }

  return updated;
}


/**
* @brief  Updates servo trim parameters.
* @param  str String containing data which to decode.
* @return true when a value has been updated.
*/
bool 
WifiConfig::updateServoTrims(StringHandler* const str)
{
  bool updated = false;

  for(int32_t i=0, j=0; i<=str->delimiter.count; i++)
  {
    str->tokens[i] = str->message.substring(j, str->delimiter.item[i]);
     j = str->delimiter.item[i] + 1;

    if(str->tokens[i].startsWith("Servo1="))
    {    
      str->tokens[i].remove(0,7);
      m_webData->servoTrim.servo1 = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->servoTrim.servo1);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("Servo2="))
    {    
      str->tokens[i].remove(0,7);
      m_webData->servoTrim.servo2 = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->servoTrim.servo2);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("Servo3="))
    {    
      str->tokens[i].remove(0,7);
      m_webData->servoTrim.servo3 = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->servoTrim.servo3);}
      updated = true;
    }
    else if(str->tokens[i].startsWith("Servo4="))
    {    
      str->tokens[i].remove(0,7);
      m_webData->servoTrim.servo4 = str->tokens[i].toInt();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->servoTrim.servo4);}
      updated = true;
    }
    else
    {
      ;
    }
  }

  return updated;
}


/**
* @brief  Updates battery scaler parameter.
* @param  str String containing data which to decode.
* @return true when a value has been updated.
*/
bool 
WifiConfig::updateBatteryTrims(StringHandler* const str)//, float* const batteryTrim)
{
  bool updated = false;

  for(int32_t i=0, j=0; i<=str->delimiter.count; i++)
  {
    str->tokens[i] = str->message.substring(j, str->delimiter.item[i]);
     j = str->delimiter.item[i] + 1;

    if(str->tokens[i].startsWith("volts="))
    { 
      str->tokens[i].remove(0,6);
      m_webData->batteryScaler = str->tokens[i].toFloat();
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println(m_webData->batteryScaler, 5);}
      updated = true;
    }
  }

  return updated;
}


/**
* @brief  Informs caller when new data has been captured.
* @return true when a value has been updated.
*/
bool 
WifiConfig::hasUpdatedData()
{
  bool updated = m_dataUpdated;
  m_dataUpdated = false;
  return updated;
}
