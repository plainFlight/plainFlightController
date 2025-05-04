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
* @file   Utilities.cpp
* @brief  This class contains methods that all clases may call upon
*/

#include "Utilities.hpp"
#include "esp_timer.h"


/**
* @brief    Proportionally maps a variable form an existing numeric range to a new numeric range. 
*           This is a faster 32 bit version when compared to Arduino's 64 bit map().
* @return   The remapped value.
*/ 
int32_t 
Utilities::map32(const int32_t x, const int32_t in_min, const int32_t in_max, const int32_t out_min, const int32_t out_max) 
{
  return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}


/**
* @brief    Calculates the loop time just passed and waits (next time in after loop execution) to give a constant loop rate.  
* @note     Using esp_timer_get_time() as it seems to save ~10us over Arduino micros().
*/ 
float
Utilities::loopRateControl()
{
  m_cycleTime = esp_timer_get_time() - m_loopStartTime;
  while(esp_timer_get_time() < m_loopEndTime);  //Lengthen last loop time if it fell short of LOOP_RATE_US

  m_lastLoopTime = m_loopStartTime;
  m_loopStartTime = esp_timer_get_time();
  m_loopEndTime = m_loopStartTime + LOOP_RATE_US; 
  m_timeDelta = static_cast<float>((m_loopStartTime - m_lastLoopTime) / 1000000.0f); 
  return m_timeDelta;
}


/**
* @brief    Outputs to console the actual looprate achieved
*/ 
void
Utilities::printLoopRateData()
{
  Serial.print(m_timeDelta, 6);
  Serial.print("\t");
  Serial.print(static_cast<float>(m_cycleTime) / 1000000.0f, 6); 
  Serial.print("\t");
  m_avCycleTime = ((m_avCycleTime * 999.0f) + static_cast<float>(m_cycleTime)) / 1000.0f;
  Serial.println(m_avCycleTime / 1000000.0f, 6);
}

/**
* @brief  A faster implementation of invSqrt.
*/
float 
Utilities::invSqrt(float x) 
{
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}


/**
* @brief  A faster implementation of atan2 when compared to C library code. Formatting left in original coding style of Author.
* @note   volkansalma/atan2_approximation.c
* @note   https://gist.github.com/volkansalma/2972237
* @note   http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
* @note   Volkan SALMA
*/
float 
Utilities::fastAtan2(float y, float x)
{
  constexpr float ONEQTR_PI = M_PI / 4.0f;
	constexpr float THRQTR_PI = 3.0f * M_PI / 4.0f;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition

	if (x < 0.0f)
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}

	angle += (0.1963f * r * r - 0.9817f) * r;

	if (y < 0.0f)
	{
      return(-angle);     // negate if in quad III or IV
  }
	else
  {
		return(angle);
  }
}