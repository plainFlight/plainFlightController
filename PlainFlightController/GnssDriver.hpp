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
* @file   GnssDriver.hpp
* @brief  An interface to dualGNSS with instatiation mechanism
*/

#pragma once

#include <type_traits>
#include "Config.hpp"
#include "dualGNSS.hpp"
// #include "UbxGNSS.hpp"
// #include "CasicGNSS.hpp"
// #include "GnssTypes.hpp"
#include "CommonTypes.hpp"

// static constexpr GnssType TYPE = GnssType::CASIC;
// static constexpr UbxSeries GENERATION = UbxSeries::UBX_M9_PLUS;

class GnssDriver
{
  public:
    bool  begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
                { return m_gnss.begin(serial, rxPin, txPin);       }
    void  beginPassive(HardwareSerial& serial, int8_t rxPin, int8_t txPin,
                    uint32_t baud = UbxConfigurator::TARGET_BAUD_RATE)
                    {m_gnss.beginPassive(serial, rxPin, txPin, baud);}
    bool  beginSafe(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
      {
        // Fast-track: attempt passive start at 115 200 baud (the library's target rate).
        // If the module is already configured and outputting valid frames, this
        // succeeds immediately and the full configuration sequence is unnecessary.
        m_gnss.beginPassive(serial, rxPin, txPin);

        const uint32_t deadline = millis() + 200UL;
        while (millis() < deadline) {
          m_gnss.update();
          if (m_gnss.hasNewData()) {
            // Fast track succeeded
            return true;  // receiving valid frames — no configuration needed
          }
        }
        // Full configuration: sweep baud rates, identify module, apply settings.
        return m_gnss.begin(serial, rxPin, txPin);
      }    
    void  update()       { m_gnss.update();      }
    bool  hasNewData()   { return m_gnss.hasNewData();  }
    bool  isFixValid()   { return m_gnss.isFixValid();  }
    void  getData(GnssData& dest)   { m_gnss.getData(dest);  }
    GnssConfigResult getConfigResult() { return m_gnss.getConfigResult(); }

  private:
    // One line. Config drives both the module type and the protocol version.
    // If GNSS_TYPE is CASIC the second argument is accepted but unused.
    Gnss<Config::GNSS_TYPE, Config::GENERATION> m_gnss;  // Configured external to the class
};