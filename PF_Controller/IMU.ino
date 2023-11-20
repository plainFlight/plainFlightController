#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "IMU.h"
#include "PIDF.h"

#define LOOP_RATE_US        (uint64_t)1000
#define I2C_CLK_1MHZ        1000000         //Over clocking by 2.5x, credit to drehmFlight for this. However, if you have I2C read/write issues change to I2C_CLK_400KHz or try reducing SDA/SCL wire length/thickness.
//#define I2C_CLK_400KHz    400000
#define MADGWICK_WARM_UP_WEIGHTING  5.0f
#define MADGWICK_FLIGHT_WEIGHTING   0.04f
#define MADGWICK_WARM_UP_LOOPS      1000U

//Instantiate required classes...
MPU6050 mpu6050;

//Module variables...
static float B_madgwick = 0.04;  //Madgwick filter parameter;

//TODO - description
void initIMU(void)
{
  Wire.begin();
  Wire.setClock(I2C_CLK_1MHZ);       //Overclocking, found this in DrehmFlight code
  
  rollPIF.begin();
  pitchPIF.begin();
  yawPIF.begin();
  
  mpu6050.initialize();

  if (mpu6050.testConnection() == false) 
  {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1);
  }
  
  Serial.println("MPU6050 initialization successful");

  //Power on reset forces all registers to 0x00, should really explicitly set them to safeguard against any issues ...but we will just set what we need
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
  mpu6050.setDLPFMode(DLPF_5HZ);
  calibrateGyro();

  madgwickWarmUp();  
}


/*
* DESCRIPTION: Calculates the loop time just passed and waits (next time in) to give a constant loop rate.
* 
*/
void loopRateControl(void)
{
  static uint64_t loopEndTime = 0U;
  static uint64_t nowTime = 0U;

  while(micros() < loopEndTime);  //Lengthen last loop time if it fell short of LOOP_RATE_US

  uint64_t lastTime = nowTime;
  nowTime = micros();
  loopEndTime = nowTime + LOOP_RATE_US;
  dt = (float)(nowTime - lastTime) / 1000000.0; 

  #ifdef DEBUG_LOOP_RATE
    Serial.println(dt, 6);
  #endif
}


//TODO - description
void madgwickWarmUp(void)
{
  static uint64_t loopEndTime = 0U;
  static uint64_t nowTime = 0U;

  //Set the accelerometer weight hight to initialise filter position quickly at power on
  B_madgwick = MADGWICK_WARM_UP_WEIGHTING;    

  //Prime the filter by running it a few times with a large accelerometer weighting
  for(uint32_t i=0U; i<MADGWICK_WARM_UP_LOOPS; i++)
  {        
    loopRateControl();
    readIMUdata();
    Madgwick6DOF(imu.gyro_X, imu.gyro_Y, imu.gyro_Z, imu.accel_X, imu.accel_Y, imu.accel_Z, dt);     
  }

  //Set the accelerometer weighting to somehting sensible for flight
  B_madgwick = MADGWICK_FLIGHT_WEIGHTING;
}


//TODO - description
void readIMUdata(void) 
{
  int16_t gyro_X, gyro_Y, gyro_Z, accel_X,accel_Y, accel_Z;

  mpu6050.getMotion6(&accel_X, &accel_Y, &accel_Z, &gyro_X, &gyro_Y, &gyro_Z);
  
  //Scale to 'g'.
  imu.accel_X = (float)accel_X / ACCEL_SCALE_FACTOR;
  imu.accel_Y = (float)accel_Y / ACCEL_SCALE_FACTOR;
  imu.accel_Z = (float)accel_Z / ACCEL_SCALE_FACTOR;

  //Gyro scaled to degrees/second. 
  imu.gyro_X = (float)(gyro_X - imu.gyroOffset_X) / GYRO_SCALE_FACTOR; 
  imu.gyro_Y = (float)(gyro_Y - imu.gyroOffset_Y) / GYRO_SCALE_FACTOR; 
  imu.gyro_Z = (float)(gyro_Z - imu.gyroOffset_Z) / GYRO_SCALE_FACTOR; 

  #ifdef MPU6050_Z_ROTATED_180
    imu.accel_X *= -1.0f;
    imu.gyro_X *= -1.0f;
    imu.accel_Y *= -1.0f;
    imu.gyro_Y *= -1.0f;   
  #endif

  #ifdef DEBUG_GYRO_DATA
    Serial.print("ax:");
    Serial.print(imu.accel_X);
    Serial.print(",\tay:");
    Serial.print(imu.accel_Y);
    Serial.print(",\taz:");
    Serial.print(imu.accel_Z);
    Serial.print(",\tgx:");
    Serial.print(imu.gyro_X);
    Serial.print(",\tgy:");
    Serial.print(imu.gyro_Y);
    Serial.print(",\tgz:");
    Serial.println(imu.gyro_Z);
  #endif
}


/*DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
*
* See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
* available (for example when using the recommended MPU6050 IMU for the default setup).
*/
//TODO - add BODMAS !!
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) 
{
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  static float q0 = 1.0f; //Initialize quaternion for madgwick filter
  static float q1 = 0.0f;
  static float q2 = 0.0f;
  static float q3 = 0.0f;
 
  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles  
  roll_IMU = fastAtan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29577951; //degrees
  pitch_IMU = asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = fastAtan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.29577951; //degrees

  #ifdef DEBUG_MADGWICK
    Serial.print("Roll: ");
    Serial.print(roll_IMU);
    Serial.print(", Pitch: ");
    Serial.print(pitch_IMU);
    Serial.print(", Yaw:");
    Serial.println(yaw_IMU);
  #endif
}


//TODO - description
float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}



/*
*volkansalma/atan2_approximation.c
*https://gist.github.com/volkansalma/2972237
*http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
*Volkan SALMA
*/
float fastAtan2(float y, float x)
{
  const float ONEQTR_PI = M_PI / 4.0;
	const float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition

	if ( x < 0.0f )
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

	if ( y < 0.0f )
	{
      return( -angle );     // negate if in quad III or IV
  }
	else
  {
		return( angle );
  }
}


//TODO - description
void calibrateGyro(void)
{
  int16_t accel_X,accel_Y, accel_Z;
  int16_t gyro_X, gyro_Y, gyro_Z;
  int64_t xGyroSum = 0;
  int64_t yGyroSum = 0;
  int64_t zGyroSum = 0;

  #ifdef DEBUG_GYRO_CALIBRATION
    Serial.println("Calibration...");
  #endif
  for(uint32_t i=0; i<CALIBRATE_COUNTS; i++)
  {
    mpu6050.getMotion6(&accel_X, &accel_Y, &accel_Z, &gyro_X, &gyro_Y, &gyro_Z);

    bool motionDetected = ((abs(gyro_X) + abs(gyro_Y) + abs(gyro_Z)) >= CALIBRATE_MAX_MOTION) ? true : false;

    if(motionDetected)
    {
      #ifdef DEBUG_GYRO_CALIBRATION
        Serial.println("Calibration reset !");
      #endif
      //craft wobbling so reset and start again
      //TODO - chance of getting stuck in here for ever if nosie threshold too low or craft wobbling/vibrating.
      i=0;
      xGyroSum = 0;
      yGyroSum = 0;
      zGyroSum = 0;
    }
    else
    {
      xGyroSum += gyro_X;
      yGyroSum += gyro_Y;
      zGyroSum += gyro_Z;
    }
  }

  //Take mean average of the sum
  imu.gyroOffset_X = (int16_t)(xGyroSum / CALIBRATE_COUNTS);
  imu.gyroOffset_Y = (int16_t)(yGyroSum / CALIBRATE_COUNTS);
  imu.gyroOffset_Z = (int16_t)(zGyroSum / CALIBRATE_COUNTS);

  #ifdef DEBUG_GYRO_CALIBRATION
    Serial.println("Calibration complete...");
    Serial.print("x: ");
    Serial.print(imu.gyroOffset_X);
    Serial.print("\ty: ");
    Serial.print(imu.gyroOffset_Y);
    Serial.print("\tz: ");
    Serial.println(imu.gyroOffset_Z);
  #endif
}