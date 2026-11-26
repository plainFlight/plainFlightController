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
#include "Monitor.hpp"


/**
* @brief  Loads the default value of every setting, used when there is no settings file.
*/
Configurator::Configurator()
{
  for (const Settings::Row& row : m_settings)
  {
    applyDefault(row);
  }
}


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
      Monitor::event(MonitorSource::CONFIGURATOR, "No file exists so create one...");
      
      if (!fileSys.createFile())
      {
        Monitor::event(MonitorSource::CONFIGURATOR, "Failed to create file");
        return false;
      }

      //Write default config data
      const bool ok2 = writeConfig();
      if (!ok2)
      {
        Monitor::event(MonitorSource::CONFIGURATOR, "Error writting Config.");
        return false;
      }
      else
      {
        Monitor::event(MonitorSource::CONFIGURATOR, "Wrote Config.");
      }

      //File data successfully written, no need to read as we wrote defaults which will be used in main program.
      return true;
    }
  } 
  else
  {
    Monitor::event(MonitorSource::CONFIGURATOR, "LitteFS failed to start ?!");
    return false;
  }   
}


/**
* @brief  Main method to handle comfiguration.
*/
void 
Configurator::operate()
{
  wifi.doWiFiStateMachine();

  if (wifi.hasUpdatedData())
  {
    const bool ok = writeConfig();
    Monitor::event(MonitorSource::CONFIGURATOR, ok ? "file write ok" : "file write failed");
  }
}


/**
* @brief  Reads the configuration file.
* @return True when configurator file successfully read.
*/
bool 
Configurator::readConfig()
{
  String fileData = "";
  const bool ok = fileSys.readDataFromFile(&fileData);
  const uint32_t configFileSize = fileData.length();

  Monitor::eventf(MonitorSource::CONFIGURATOR, "File size: %lu", configFileSize);
  Monitor::event(MonitorSource::CONFIGURATOR, fileData.c_str());

  if (!ok)
  {
    Monitor::event(MonitorSource::CONFIGURATOR, "Failed to read Json from file");
    return false;
  }
  else
  {
    Monitor::event(MonitorSource::CONFIGURATOR, "Read Json from file :-)");
    // Allocate a temporary JsonDocument
    JsonDocument jsonDoc;
    // Deserialize the JSON document    
    DeserializationError error = deserializeJson(jsonDoc, fileData);
    if (error)
    {
      Monitor::event(MonitorSource::CONFIGURATOR, "Json deserializeJson error");
      return false;
    }
    else
    {
      const JsonDocument& constDoc = jsonDoc;
      bool rewrite = false;

      const uint32_t fileSchema = constDoc[KEY_SCHEMA] | 0U;
      const bool newerSchema = (fileSchema > Settings::SCHEMA);
      if (newerSchema)
      {
        Monitor::eventf(MonitorSource::CONFIGURATOR, "File schema %lu is newer than %lu, file not rewritten at boot", fileSchema, Settings::SCHEMA);
      }
      else if (fileSchema < Settings::SCHEMA)
      {
        Monitor::eventf(MonitorSource::CONFIGURATOR, "File schema %lu is older than %lu, file will be rewritten", fileSchema, Settings::SCHEMA);
        rewrite = true;
      }

      //decode file data
      for (const Settings::Row& row : m_settings)
      {
        if (readSetting(row, constDoc[row.key]))
        {
          rewrite = true;
        }
      }

      //Make the file complete, unless it came from newer firmware whose settings should survive
      if (rewrite && !newerSchema)
      {
        const bool written = writeConfig();
        Monitor::event(MonitorSource::CONFIGURATOR, written ? "File rewritten" : "File rewrite failed");
      }
      return true;
    }
  }
}


/**
* @brief  Writes the configuration file.
* @return True when configurator file successfully written.
*/
bool 
Configurator::writeConfig()
{
  JsonDocument jsonDoc;

  for (const Settings::Row& row : m_settings)
  {
    if (nullptr != row.storage.intValue)
    {
      jsonDoc[row.key] = *row.storage.intValue;
    }
    else
    {
      jsonDoc[row.key] = *row.storage.floatValue;
    }
  }
 
  jsonDoc[KEY_SCHEMA] = Settings::SCHEMA;

  String jsonString = "";
  serializeJson(jsonDoc, jsonString);
  const bool ok = fileSys.writeDataToFile(&jsonString);

  if (ok)
  {
    return true;
  }

  Monitor::event(MonitorSource::CONFIGURATOR, "Failed to write Json to file");
  return false;
}


/**
* @brief  Loads the default value of one setting.
* @param  row The setting.
*/
void
Configurator::applyDefault(const Settings::Row& row)
{
  if (nullptr != row.storage.intValue)
  {
    *row.storage.intValue = Settings::toStoredInt(row.defaultValue, row.scale);
  }
  else
  {
    *row.storage.floatValue = Settings::toStoredFloat(row.defaultValue, row.scale);
  }
}


/**
* @brief  Loads one setting from the settings file.
* @param  row        The setting.
* @param  fileValue  Its value in the settings file, null when the key is missing.
* @return True when the value in the file was missing, invalid or out of range, so the file needs rewriting.
* @note   A missing, wrong type, non-finite or out of range value loads the default.
*/
bool
Configurator::readSetting(const Settings::Row& row, JsonVariantConst fileValue)
{
  if (nullptr != row.storage.intValue)
  {
    if (!fileValue.is<int32_t>())
    {
      applyDefault(row);
      Monitor::eventf(MonitorSource::CONFIGURATOR, "%s missing or not a whole number, default %ld loaded", row.key, *row.storage.intValue);
      return true;
    }

    const int32_t value = fileValue.as<int32_t>();
    const int32_t min = Settings::toStoredInt(row.min, row.scale);
    const int32_t max = Settings::toStoredInt(row.max, row.scale);

    if (Settings::isInRange(value, min, max))
    {
      *row.storage.intValue = value;
      return false;
    }

    applyDefault(row);
    Monitor::eventf(MonitorSource::CONFIGURATOR, "%s = %ld is outside %ld to %ld, default %ld loaded", row.key, value, min, max, *row.storage.intValue);
    return true;
  }
  else
  {
    if (!fileValue.is<float>() || !std::isfinite(fileValue.as<float>()))
    {
      applyDefault(row);
      Monitor::eventf(MonitorSource::CONFIGURATOR, "%s missing or not a finite number, default %.5f loaded", row.key, *row.storage.floatValue);
      return true;
    }

    const float value = fileValue.as<float>();
    const float min = Settings::toStoredFloat(row.min, row.scale);
    const float max = Settings::toStoredFloat(row.max, row.scale);

    if (Settings::isInRange(value, min, max))
    {
      *row.storage.floatValue = value;
      return false;
    }

    applyDefault(row);
    Monitor::eventf(MonitorSource::CONFIGURATOR, "%s = %.5f is outside %.5f to %.5f, default %.5f loaded", row.key, value, min, max, *row.storage.floatValue);
    return true;
  }
}
