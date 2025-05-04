/* 
* Copyright (c) 2023,2025 P.Cook (alias 'plainFlight')
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

/*
* PIDF class.
*/
#pragma once

#include <inttypes.h>




class PIDF
{
  public:  
    struct Gains
    {
      int32_t p;
      int32_t i;
      int32_t d;
      int32_t ff;
    };

    struct AxisGains
    {
      Gains pitch;
      Gains roll;
      Gains yaw;
    };

    static constexpr int64_t PIDF_MAX_LIMIT         = 10000;  //This is just to limit the maximum PIDF output

    PIDF(){};
    ~PIDF(){};
    int32_t pidfController(const int32_t setPoint, const int32_t actualPoint, const Gains* const gains);
    int32_t getPidError();
    void setPGain(int32_t pGain);
    void setIGain(int32_t iGain);
    void setDGain(int32_t dGain);
    void setFGain(int32_t fGain);
    void iTermReset();

   private: 
    //Debug defines
    static constexpr bool DEBUG_PID                 = false;

    static constexpr int64_t I_TERM_DENOMINATOR     = 1000;    
    static constexpr int64_t F_TERM_DENOMINATOR     = 100;
    static constexpr int64_t PIDF_TERM_DENOMINATOR  = 100; 
    //PID wind up limits
    static constexpr int64_t I_WIND_UP_LIMIT        = 1000000;
    static constexpr int64_t D_WIND_UP_LIMIT        = 500000;

    uint32_t m_kP;
    uint32_t m_kI;
    uint32_t m_kD;
    uint32_t m_kF;      
    int32_t m_error = 0;
    int64_t m_pTerm = 0;
    int64_t m_iTerm = 0;
    int64_t m_dTerm = 0;  
    int64_t m_fTerm = 0; 
    int64_t m_dLastError = 0;
    int64_t m_pidTerm = 0;
    int64_t m_iGainWindUp = I_WIND_UP_LIMIT;
    int64_t m_dTermMaxLimit = D_WIND_UP_LIMIT;
};

