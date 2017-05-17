#include <Wire.h>


#define MPU_addr 0x68
#define accelP 0x1C
#define gyroP 0x1B
//const int gyro[3] = {0x43, 0x45, 0x47}; //x, y, z for the high byte
//const int accel[3] = {0x3B, 0x3D, 0x3F};

#define gyroData_StartAddress 0x43
#define accelData_StartAddress 0x3B

#define sleepAddress 0x6B


void setSleep(bool enable){
  Wire.beginTransmission(MPU_addr);
  Wire.write(sleepAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 1, true);
  uint8_t power = Wire.read();
  power &= 0b10111111;
  power |= enable<<6;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(power);
  Wire.endTransmission(true);
}

void getAccelData( uint16_t* ax,uint16_t* ay, uint16_t* az){
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(accelData_StartAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); //2 bytes for X, 2 bytes for Y, 2 bytes for Z
  *ax = Wire.read() << 8 | Wire.read(); //read bytes 1 and 2
  *ay = Wire.read() << 8 | Wire.read(); //read bytes 3 and 4
  *az = Wire.read() << 8 | Wire.read(); //read bytes 5 and 6
  
//  for(int j = 0; j < 3; j++){
//    Wire.beginTransmission(MPU_addr);
//    Wire.write(accel[j]);
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU_addr, 2, true);
//    if(j == 0){
//      *ax = Wire.read() << 8 | Wire.read();
//    } else if(j == 1){
//      *ay = Wire.read() << 8 | Wire.read();
//    } else {
//      *az = Wire.read() << 8 | Wire.read();
//    }
//  }
}

void getGyroData( uint16_t* gx, uint16_t* gy, uint16_t* gz){

  Wire.beginTransmission(MPU_addr);
  Wire.write(gyroData_StartAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); //2 bytes for X, 2 bytes for Y, 2 bytes for Z
  *gx = Wire.read() << 8 | Wire.read(); //read bytes 1 and 2
  *gy = Wire.read() << 8 | Wire.read(); //read bytes 3 and 4
  *gz = Wire.read() << 8 | Wire.read(); //read bytes 5 and 6
  
//  for(int j = 0; j < 3; j++){
//    Wire.beginTransmission(MPU_addr);
//    Wire.write(gyro[j]);
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU_addr, 2, true);
//    if(j == 0){
//      *gx = Wire.read() << 8 | Wire.read();
//    } else if(j == 1){
//      *gy = Wire.read() << 8 | Wire.read();
//    } else {
//      *gz = Wire.read() << 8 | Wire.read();
//    }
//  }
}

void setGyroPres(uint8_t val){
  val &=0b11;
  val= val<<3;
  Wire.beginTransmission(MPU_addr);
  Wire.write(gyroP);
  Wire.write(val);
  Wire.endTransmission(true);
}

void setAccelPres(uint8_t val){
  val &=0b11;
  val= val<<3;
  Wire.beginTransmission(MPU_addr);
  Wire.write(accelP);
  Wire.write(val);
  Wire.endTransmission(true);
}

uint16_t a_data_x = 0;
uint16_t a_data_y = 0;
uint16_t a_data_z = 0;

uint16_t g_data_x = 0;
uint16_t g_data_y = 0;
uint16_t g_data_z = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  Serial.println("started serial");
  
  Wire.begin();
  
  setSleep(false);
  setGyroPres(2);
  setAccelPres(2);
}

void loop()
{
  getAccelData(&a_data_x, &a_data_y, &a_data_z);
  
  getGyroData(&g_data_x, &g_data_y, &g_data_z);
  
  
  Serial.print("Accelerometer: ");
//  Serial.println(a_data_x);
  Serial.println((int16_t)a_data_x);
//  Serial.print(", ");
//  Serial.print(a_data_y);
//  Serial.print(", ");
//  Serial.println(a_data_z);
  
  /*Serial.print("Gyroscope: ");
  Serial.print(g_data_x);
  Serial.print(", ");
  Serial.print(g_data_y);
  Serial.print(", ");
  Serial.println(g_data_z);

  Serial.println("-----------------------");
  */
}


