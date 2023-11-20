

#include "PIDF.h"
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif


PIDF::PIDF(int32_t iGainWindUp, int32_t dTermMaxLimit)
{
  this->iGainWindUp = iGainWindUp;  
  this->dTermMaxLimit = dTermMaxLimit;
}


void PIDF::begin(void)
{     
  this->error = 0;
  this->pTerm = 0;
  this->iTerm = 0;
  this->dTerm = 0;  
  this->fTerm = 0; 
  this->dLastError = 0;
  this->pidTerm = 0;
}


int32_t PIDF::pidfController(int32_t setPoint, int32_t actualPoint, const Gains* const gains)
{
  this->error = setPoint - actualPoint;

  //P Term calculations
  this->pTerm = (int64_t)(this->error * (int64_t)gains->p);
  this->iTerm += ((int64_t)this->error * (int64_t)gains->i) / I_TERM_DENOMINATOR;
  
  if(abs(this->iTerm) > this->iGainWindUp)
  {
    //Limit iTerm to what we need
    if(0 > this->iTerm)
    {
      //iTerm gone negative
      this->iTerm = 0 - iGainWindUp;
    }
    else
    {
      this->iTerm = this->iGainWindUp;
    }
  }
 
  //D Term calculations
  this->dTerm = ((int64_t)this->error - this->dLastError) * (int64_t)gains->d;
  
  if(abs(this->dTerm) > this->dTermMaxLimit)
  {
    if(0 > this->dTerm)
    {
      //iTerm negative
      this->dTerm = 0 - this->dTermMaxLimit;
    }
    else
    {
      this->dTerm = this->dTermMaxLimit;
    }
  }
   
  this->dLastError = (int64_t)this->error;   

  this->fTerm = ((int64_t)setPoint * (int64_t)gains->ff);// / F_TERM_DENOMINATOR;

  //Sum PID terms
  this->pidTerm =  (this->pTerm + this->iTerm + this->dTerm + this->fTerm) / PIDF_TERM_DENOMINATOR; //Need to add 50 to the sum before divide by 100 to improve rounding errors
      
  #ifdef DEBUG_PID
    Serial.print(setPoint);
    Serial.print(",");
    Serial.print(actualPoint);
    Serial.print(",");
    Serial.print(this->pTerm);
    Serial.print(",");
    Serial.print(this->iTerm);
    Serial.print(",");
    Serial.print(this->dTerm);
    Serial.print(",");
    Serial.print(this->fTerm);
    Serial.print(",");
    Serial.println(this->pidTerm); 
  #endif 

  return CROP_PIDF((int32_t)this->pidTerm);
  //return (int32_t)this->pidTerm;
}




int32_t PIDF::getPidError(void)
{
  return this->error;
}

int32_t PIDF::getPTerm(void)
{
  return this->pTerm;
}

int32_t PIDF::getITerm(void)
{
  return this->iTerm;
}

int32_t PIDF::getDTerm(void)
{
  return this->dTerm;
}

int32_t PIDF::getPGain(void)
{
  return this->kP;
}

int32_t PIDF::getIGain(void)
{
  return this->kI;
}

int32_t PIDF::getDGain(void)
{
  return this->kD;
}

int32_t PIDF::getFGain(void)
{
  return this->kF;
}

void PIDF::setPGain(int32_t pGain)
{
  this->kP = pGain;
}

void PIDF::setIGain(int32_t iGain)
{
  this->kI = iGain;
}

void PIDF::setDGain(int32_t dGain)
{
  this->kD = dGain;
}

void PIDF::setFGain(int32_t fGain)
{
  this->kF = fGain;
}

void PIDF::iTermReset(void)
{
  this->iTerm = 0;
}
