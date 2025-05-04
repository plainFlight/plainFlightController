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
* @file   Configurator.cpp
* @brief  This class contains methods that controls the configurator, including FileSystem and WifiConfig.
*/

#include "Configurator.hpp"


/**
* @brief  Start the configurator process/object.
* @return True when configurator successfully started.
*/
bool
Configurator::begin()
{
  bool ok = fileSys.begin(); 

  if (ok)
  {
    if (fileSys.fileExists())
    {
      //File exists so get configuration data
      ok = readConfig();
      return (ok) ? true : false;
    }
    else
    {      
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("No file exists so create one...");}
      
      if (!fileSys.createFile())
      {
        if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Failed to create file");}
        return false;
      }

      //Write default config data
      const bool ok2 = writeConfig();
      if (!ok2)
      {
        if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Error writting Config.");}
        return false;
      }
      else
      {
        if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Wrote Config.");}
      }

      //File data successfully written, no need to read as we wrote defaults which will be used in main program.
      return true;
    }
  } 
  else
  {
    if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("LitteFS failed to start ?!");}
    return false;
  }   
}


void 
Configurator::operate()
{
  wifi.doWiFiStateMachine();

  if (wifi.hasUpdatedData())
  {
    const bool ok = writeConfig();

    if constexpr(Config::DEBUG_CONFIGURATOR)
    {
      if (ok)
      {
        Serial.println("file write ok");
      }
      else
      {
        Serial.println("file write failed");
      }
    }
  }
}


bool 
Configurator::readConfig()
{
  String fileData = "";
  const bool ok = fileSys.readDataFromFile(&fileData);
  const uint32_t configFileSize = fileData.length();

  if constexpr(Config::DEBUG_CONFIGURATOR)
  {
    Serial.print("File size: ");
    Serial.println(configFileSize);
    Serial.println(fileData);
  }

  if (!ok)
  {
    if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Failed to read Json from file");}
    return false;
  }
  else
  {
    if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Read Json from file :-)");}
    // Allocate a temporary JsonDocument
    JsonDocument jsonDoc;
    // Deserialize the JSON document    
    DeserializationError error = deserializeJson(jsonDoc, fileData);
    if (error)
    {
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Json deserializeJson error");}
      return false;
    }
    else
    {
      //decode file data
      m_fileData.gains.pitch.p = jsonDoc["Pitch P"];
      m_fileData.gains.pitch.i = jsonDoc["Pitch I"];
      m_fileData.gains.pitch.d = jsonDoc["Pitch D"];
      m_fileData.gains.pitch.ff = jsonDoc["Pitch F"];

      m_fileData.gains.roll.p = jsonDoc["Roll P"];
      m_fileData.gains.roll.i = jsonDoc["Roll I"];
      m_fileData.gains.roll.d = jsonDoc["Roll D"];
      m_fileData.gains.roll.ff = jsonDoc["Roll F"];

      m_fileData.gains.yaw.p = jsonDoc["Yaw P"];
      m_fileData.gains.yaw.i = jsonDoc["Yaw I"];
      m_fileData.gains.yaw.d = jsonDoc["Yaw D"];
      m_fileData.gains.yaw.ff = jsonDoc["Yaw F"];

      m_fileData.rates.pitch = jsonDoc["Pitch Rate"];
      m_fileData.rates.roll = jsonDoc["Roll Rate"];
      m_fileData.rates.yaw = jsonDoc["Yaw Rate"];

      m_fileData.maxAngle.pitch = jsonDoc["Pitch Angle"];
      m_fileData.maxAngle.roll = jsonDoc["Roll Angle"];

      m_fileData.levelTrim.pitch  = jsonDoc["Pitch Trim"];
      m_fileData.levelTrim.roll   = jsonDoc["Roll Trim"];
      m_fileData.levelTrim.yaw    = jsonDoc["Yaw Trim"];
      m_fileData.servoTrim.servo1 = jsonDoc["Servo1 Trim"];
      m_fileData.servoTrim.servo2 = jsonDoc["Servo2 Trim"];
      m_fileData.servoTrim.servo3 = jsonDoc["Servo3 Trim"];
      m_fileData.servoTrim.servo4 = jsonDoc["Servo4 Trim"];

      m_fileData.batteryScaler = jsonDoc["Batt Scale"];
      return true;
    }
  }
}


bool 
Configurator::writeConfig()
{
  JsonDocument jsonDoc;

  //decode file data
  jsonDoc["Pitch P"] = m_fileData.gains.pitch.p;
  jsonDoc["Pitch I"] = m_fileData.gains.pitch.i;
  jsonDoc["Pitch D"] = m_fileData.gains.pitch.d;
  jsonDoc["Pitch F"] = m_fileData.gains.pitch.ff;

  jsonDoc["Roll P"] = m_fileData.gains.roll.p;
  jsonDoc["Roll I"] = m_fileData.gains.roll.i;
  jsonDoc["Roll D"] = m_fileData.gains.roll.d;
  jsonDoc["Roll F"] = m_fileData.gains.roll.ff;

  jsonDoc["Yaw P"] = m_fileData.gains.yaw.p;
  jsonDoc["Yaw I"] = m_fileData.gains.yaw.i;
  jsonDoc["Yaw D"] = m_fileData.gains.yaw.d;
  jsonDoc["Yaw F"] = m_fileData.gains.yaw.ff;

  jsonDoc["Pitch Rate"] = m_fileData.rates.pitch;
  jsonDoc["Roll Rate"] = m_fileData.rates.roll;
  jsonDoc["Yaw Rate"] = m_fileData.rates.yaw;

  jsonDoc["Pitch Angle"] = m_fileData.maxAngle.pitch;
  jsonDoc["Roll Angle"] = m_fileData.maxAngle.roll;

  jsonDoc["Pitch Trim"] = m_fileData.levelTrim.pitch;
  jsonDoc["Roll Trim"] = m_fileData.levelTrim.roll;
  jsonDoc["Yaw Trim"] = m_fileData.levelTrim.yaw;
  jsonDoc["Servo1 Trim"] = m_fileData.servoTrim.servo1;
  jsonDoc["Servo2 Trim"] = m_fileData.servoTrim.servo2;
  jsonDoc["Servo3 Trim"] = m_fileData.servoTrim.servo3;
  jsonDoc["Servo4 Trim"] = m_fileData.servoTrim.servo4;

  jsonDoc["Batt Scale"] = m_fileData.batteryScaler;

  String jsonString = "";
  serializeJson(jsonDoc, jsonString);
  const bool ok = fileSys.writeDataToFile(&jsonString);

  if (ok)
  {
    return true;
  }

  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Failed to write Json to file");}
  return false;
}

