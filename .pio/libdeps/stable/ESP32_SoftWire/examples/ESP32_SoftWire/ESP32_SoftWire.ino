#include "ESP32_SoftWire.h"

SoftWire i2c;

const int i2c_SDA_PIN  = 23; // 0-31
const int i2c_SCL_PIN  = 22; // 0-31

/* Example output
===================
ESP32 SoftWire Demo
===================
I2C: Scanning ...
I2C: Found address: 0x1E (decimal 30)
     adr=0x1E reg=0x0A -> 0x483433 ---> FOUND: HMC5883L
I2C: Found address: 0x76 (decimal 118)
     adr=0x76 reg=0xD0 -> 0x58 ---> FOUND: BMP280
I2C: Found 2 device(s)
*/

void setup() {
  Serial.begin(115200);
  i2c.begin(i2c_SDA_PIN, i2c_SCL_PIN, 400000);
}

void loop() {
  Serial.println("===================\nESP32 SoftWire Demo\n===================");

  Serial.printf("I2C: Scanning ...\n");
  int num_adr_found = 0;
  i2c.begin();
  for (uint8_t adr = 8; adr < 120; adr++) {
    i2c.beginTransmission(adr);          // Begin I2C transmission Address (i)
    if (i2c.endTransmission() == 0) { // Receive 0 = success (ACK response) 
      Serial.printf("I2C: Found address: 0x%02X (decimal %d)\n",adr,adr);
      num_adr_found++;
      i2c_identify_chip(adr);
    }
  }
  Serial.printf("I2C: Found %d device(s)\n", num_adr_found);
  delay(1000);
}

struct test_struct{
    uint8_t adr1;
    uint8_t adr2;    
    uint8_t reg;
    int len;
    uint32_t expected;
    char descr[60];
};

//table of addresses, register and expected reply
test_struct tests[] = {
// adr1  adr2  reg  len exp       descr
  {0x1E, 0x1E, 0x0A, 3, 0x483433, "HMC5883L"}, // ID-Reg-A 'H', ID-Reg-B '4', ID-Reg-C '3'
  {0x68, 0x69, 0x00, 1, 0x68, "MPU3050"},
  {0x68, 0x69, 0x00, 1, 0x69, "MPU3050"},
  {0x68, 0x69, 0x75, 1, 0x68, "MPU6050"},
  {0x68, 0x69, 0x75, 1, 0x72, "FAKE MPU6050, got WHO_AM_I=0x72, real chip returns 0x68"},
  {0x68, 0x69, 0x75, 1, 0x19, "MPU6886"},
  {0x68, 0x69, 0x75, 1, 0x68, "MPU9150"},
  {0x68, 0x69, 0x75, 1, 0x70, "MPU6500"},
  {0x68, 0x69, 0x75, 1, 0x71, "MPU9250"},
  {0x68, 0x69, 0x75, 1, 0x98, "ICM-20689"},
  {0x76, 0x76, 0xD0, 1, 0x56, "BMP280"},
  {0x76, 0x76, 0xD0, 1, 0x57, "BMP280"},
  {0x76, 0x76, 0xD0, 1, 0x58, "BMP280"},
  {0x76, 0x76, 0xD0, 1, 0x60, "BME280"},
  {0x76, 0x76, 0xD0, 1, 0x61, "BMP680"},
  {0x40, 0x4F, 0x00, 2, 0x399F, "INA219"}, //adr 0x40-0x4F, reg 0x00->0x399F
  {0x40, 0x4F, 0x00, 2, 0x4127, "INA226"}, //adr 0x40-0x4F, reg 0x00->0x4127
  {0x48, 0x4F, 0x01, 2, 0x8583, "ADS1113, ADS1114, or ADS1115" },
  {0x77, 0x77, 0, 0, 0, "MS5611"}, //identify by address match only 
  {0,0,0,0,0,""} //end
};

void i2c_identify_chip(uint8_t adr) {
  int i = 0;
  while(tests[i].adr1) {
    int adr1 = tests[i].adr1;
    int adr2 = tests[i].adr2;    
    int reg = tests[i].reg;
    int len = tests[i].len;    
    int expected = tests[i].expected;
    uint32_t received = 0;
    bool match = (len == 0 && adr1<=adr && adr<=adr2);
    if(!match && adr1<=adr && adr<=adr2) {
      uint8_t data[4];
      i2c_ReadRegs(adr, reg, data, len);
      for(int i=0;i<len;i++) received = (received<<8) + data[i];
      match = (received == expected);
    }
    if(match) {
      Serial.printf("     adr=0x%02X reg=0x%02X -> 0x%02X ---> FOUND: %s\n", adr, reg, received, tests[i].descr);
    }
    i++;
  }
}

void WriteReg( uint8_t adr, uint8_t reg, uint8_t data ) {
  i2c.beginTransmission(adr); 
  i2c.write(reg);       
  i2c.write(data);              
  i2c.endTransmission();
}

unsigned int i2c_ReadReg( uint8_t adr, uint8_t reg ) {
    uint8_t data = 0;
    i2c_ReadRegs(adr, reg, &data, 1);
    return data;
}

void i2c_ReadRegs( uint8_t adr, uint8_t reg, uint8_t *data, uint8_t n ) {
  i2c.beginTransmission(adr); 
  i2c.write(reg);
  i2c.endTransmission(false); //false = repeated start
  uint8_t bytesReceived = i2c.requestFrom(adr, n);
  if(bytesReceived == n) {
    i2c.readBytes(data, bytesReceived);
  }
}

void i2c_scan() {
  Serial.printf("I2C: Scanning ...\n");
  byte count = 0;
  i2c.begin();
  for (byte i = 8; i < 120; i++) {
    i2c.beginTransmission(i);          // Begin I2C transmission Address (i)
    if (i2c.endTransmission() == 0) { // Receive 0 = success (ACK response) 
      Serial.printf("I2C: Found address: 0x%02X (%d)\n",i,i);
      count++;
    }
  }
  Serial.printf("I2C: Found %d device(s)\n", count);      
}
