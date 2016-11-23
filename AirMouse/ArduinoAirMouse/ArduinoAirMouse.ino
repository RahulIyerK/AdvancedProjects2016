#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"
#include <Wire.h>

RF24 radio(9,10);

uint64_t pipes[2] = {0xF0BADA5535, 0xF0B16B00B5}; //reading, writing

void initRadio()
{
  radio.setPALevel(RF24_PA_HIGH);
  //payload size default 32...
  radio.setChannel(10); //we're team 10 :) 
  radio.setCRCLength(RF24_CRC_16); //2-byte CRC
  radio.setDataRate(RF24_1MBPS); //1Mbps data rate

  radio.openReadingPipe(0, pipes[0]);
  radio.openWritingPipe(pipes[1]);
}

const int buttonPinR = 5;
const int buttonPinL = 3;

const int MPU_addr = 0x68;
const int accelP = 0x1C;
const int gyroP = 0x1B;
const int gyro[3] = {0x43, 0x45, 0x47}; //x, y, z for the high byte
const int accel[3] = {0x3B, 0x3D, 0x3F};
const int sleepAddress = 0x6B;


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
  for(int j = 0; j < 3; j++){
    Wire.beginTransmission(MPU_addr);
    Wire.write(accel[j]);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);
    if(j == 0){
      *ax = Wire.read() << 8 | Wire.read();
    } else if(j == 1){
      *ay = Wire.read() << 8 | Wire.read();
    } else {
      *az = Wire.read() << 8 | Wire.read();
    }
  }
}

void getGyroData( uint16_t* gx, uint16_t* gy, uint16_t* gz){
  
  for(int j = 0; j < 3; j++){
    Wire.beginTransmission(MPU_addr);
    Wire.write(gyro[j]);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);
    if(j == 0){
      *gx = Wire.read() << 8 | Wire.read();
    } else if(j == 1){
      *gy = Wire.read() << 8 | Wire.read();
    } else {
      *gz = Wire.read() << 8 | Wire.read();
    }
  }
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("started serial");
  
  pinMode(buttonPinR, INPUT);
  pinMode(buttonPinL, INPUT);

  radio.begin();
  initRadio();
  Wire.begin();
  radio.stopListening();
  setSleep(false);
  setGyroPres(2);
  setAccelPres(2);
}

long lastDebounceTimeR = 0;  // the last time the output pin was toggled
long lastDebounceTimeL = 0;
long debounceDelay = 50;
int buttonStateR = LOW;             // the current reading from the input pin
int buttonStateL = LOW;
int lastReadingR = LOW;
int lastReadingL = LOW;

const double VOLTAGE_ADC_RATIO = (3.496 / 1023) * ((3.3 + 1)/3.3);

char readButtonR(){
  int reading = digitalRead(buttonPinR);//get what state the button is
  char out = 'a';//the value to return if nothing special happened
  
  if (reading != lastReadingR) {//We're reading a new state for button
    // reset the debouncing timer
    lastDebounceTimeR = millis();
  }

  if ((millis() - lastDebounceTimeR) > debounceDelay) {//We finally have a stable value
    if (reading != buttonStateR)//Compared to our previous state, we have a flip
    {
      out = 'r';//prepare to toggle the Mini
    }
    buttonStateR = reading;//Make the buttonState the same
  }

  lastReadingR = reading;//make the last state the "current" state
  return out;
}

char readButtonL(){
  int reading = digitalRead(buttonPinL);
  char out = 'a';
  
  if (reading != lastReadingL) {
    // reset the debouncing timer
    lastDebounceTimeL = millis();
  }

  if ((millis() - lastDebounceTimeL) > debounceDelay) {
    
    if (reading != buttonStateL)
    {
      out = 'l';
    }
    buttonStateL = reading;
  }

  lastReadingL = reading;
  return out;
}

struct data
{
  boolean isPushedR = false;
  boolean isPushedL = false;
  uint16_t batteryADC = 0;
  uint16_t acceleration[3] = {0, 0, 0};
  uint16_t gyroscope[3] = {0, 0, 0};
};

data packet;
boolean rState = false;//these states are used to represent the current state of the buttons
boolean lState = false;

void loop() {    
    
    if(readButtonR() == 'r'){
      rState = !rState;
    }
    if(readButtonL() == 'l'){
      lState=!lState;
    }
  
    if (rState == true) {
      packet.isPushedR = true;
    }
    else {
      packet.isPushedR = false;
    }
    if (lState == true) {
      packet.isPushedL = true;
    }
    else {
      packet.isPushedL = false;
    }
    Serial.println("Finished reading in presses");

    int vD = analogRead(1);
    Serial.println(vD);
    Serial.println();
    
    uint16_t voltage_DividerOut = (uint16_t)analogRead(1);
    packet.batteryADC = voltage_DividerOut;

    getGyroData(packet.gyroscope, packet.gyroscope+1, packet.gyroscope+2);
    getAccelData(packet.acceleration, packet.acceleration+1, packet.acceleration+2);
    
    Serial.println("Starting to write");
    radio.write((char*) &packet, sizeof(packet));
    Serial.println(packet.batteryADC);
    Serial.println("Finished writing");
}
