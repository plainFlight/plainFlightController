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
* @file   FileSystem.cpp
* @brief  This class contains methods that controls the reading and writting of flash memory.
*/

#include "FileSystem.hpp"


bool 
FileSystem::begin()
{
  String filePath = FILE_DIRECTORY + "/" + FILE_NAME; 
  stringToCharArray(&filePath, m_filePath);
  stringToCharArray(&FILE_DIRECTORY, m_fileDirectory);

  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED, m_fileDirectory, 1))  
  {
    return false;
  }

  return true;
}


bool 
FileSystem::fileExists()
{
  return (LittleFS.exists(m_filePath)) ? true : false;
}


bool 
FileSystem::createFile()
{
  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Formating.");}
  LittleFS.format();
  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Create directory.");}
  LittleFS.mkdir(FILE_DIRECTORY);

  if (!LittleFS.exists(FILE_DIRECTORY)) 
  {
      if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("directory not created");}
      return false;
  }

  return true;
}


void
FileSystem::stringToCharArray(const String* const str, char* const charBuff)
{
  uint32_t strLength = str->length() + 1U;
  str->toCharArray(charBuff, strLength);
}


bool 
FileSystem::readDataFromFile(String *const fileData)
{
  File file = LittleFS.open(m_filePath);

  if (!file || file.isDirectory()) 
  {
    if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("- failed to open file for reading");}
    return false;
  }

  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("- read from file:");}

  while (file.available()) 
  {
    *fileData = file.readString();
  }

  file.close();
  return true;
}


bool 
FileSystem::writeDataToFile(String const * const fileData)
{
  File file = LittleFS.open(m_filePath, FILE_WRITE);

  if (!file) 
  {
    if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("Failed to open file for writing");}
    return false;
  }
  
  //write default gains here
  if constexpr(Config::DEBUG_CONFIGURATOR){Serial.println("write default gains");}
  file.print(*fileData);
  file.close();
  return true;
}