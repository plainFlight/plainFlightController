/* 
* Copyright (c) 2023,2024 P.Cook (alias 'plainFlight')
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
* PIDF class originally written by plainFlight for balancing robot project.
*/

#ifndef PIDF_h
#define PIDF_h

#include <inttypes.h>


#define I_TERM_DENOMINATOR    (int64_t)1000    
#define F_TERM_DENOMINATOR    (int64_t)100
#define PIDF_TERM_DENOMINATOR (int64_t)100
#define PIDF_MAX_LIMIT        (int32_t)10000  //This is just to limit the maximum PIDF output 
//Inline macro to limit IMU calculations
#define CROP_PIDF(x)          (x < -PIDF_MAX_LIMIT) ? -PIDF_MAX_LIMIT: ((x > PIDF_MAX_LIMIT) ? PIDF_MAX_LIMIT: x)

typedef struct
{
  int32_t p;
  int32_t i;
  int32_t d;
  int32_t ff;
}Gains;

class PIDF 
{
  private: 
    uint32_t kP;
    uint32_t kI;
    uint32_t kD;
    uint32_t kF;      
    int32_t error;
    int64_t pTerm;
    int64_t iTerm;
    int64_t dTerm;  
    int64_t fTerm; 
    int64_t dLastError;
    int64_t pidTerm;
    int64_t iGainWindUp;
    int64_t dTermMaxLimit;
    
  public:  

    PIDF(int32_t iGainWindUp, int32_t dTermMaxLimit);
    void begin(void);
    int32_t pidfController(int32_t setPoint, int32_t actualPoint, const Gains* const gains);
    int32_t getPidError(void);
    int32_t getPTerm(void);
    int32_t getITerm(void);
    int32_t getDTerm(void);
    int32_t getPGain(void);
    int32_t getIGain(void);
    int32_t getDGain(void);
    int32_t getFGain(void);
    void setPGain(int32_t pGain);
    void setIGain(int32_t iGain);
    void setDGain(int32_t dGain);
    void setFGain(int32_t fGain);
    void iTermReset(void);
};

#endif
