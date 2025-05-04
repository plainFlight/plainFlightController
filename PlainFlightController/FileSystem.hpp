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
* @file   FileSystem.hpp
* @brief  This class contains methods that controls the reading and writting of flash memory.
*/

#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include "FS.h"
#include "PIDF.hpp"
#include "Config.hpp"

class FileSystem
{
  public:
    struct Rates  
    {
      int32_t pitch;
      int32_t roll;
      int32_t yaw;
    };

    struct LevelTrims  
    {
      float pitch;
      float roll;
      float yaw;
    };

    struct ServoTrims  
    {
      int32_t servo1;
      int32_t servo2;
      int32_t servo3;
      int32_t servo4;
    };

    struct MaxAngle  
    {
      int32_t pitch;
      int32_t roll;
    };

    struct  NonVolatileData
    {
      PIDF::AxisGains gains;
      Rates rates;
      MaxAngle maxAngle;
      LevelTrims levelTrim;
      ServoTrims servoTrim;
      float batteryScaler;
    };

    FileSystem(){};
    bool begin();
    bool readDataFromFile(String *const fileData);
    bool writeDataToFile(String const * const fileData);
    bool fileExists();
    bool createFile();
    
  private:
    static constexpr bool FORMAT_LITTLEFS_IF_FAILED = true;
    static constexpr uint8_t MAX_FILE_PATH_SIZE = 50U;
    static constexpr uint8_t MAX_FILE_DIRECTORY_SIZE = 25U;
    const String FILE_DIRECTORY = "/plainFlight";
    const String FILE_NAME = "plainFlightController.txt";

    char m_filePath[MAX_FILE_PATH_SIZE];
    char m_fileDirectory[MAX_FILE_DIRECTORY_SIZE];
    
    void stringToCharArray(const String* const str, char* const charBuff);
};