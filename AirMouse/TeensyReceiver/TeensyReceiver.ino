#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"

RF24 radio(9,10);

uint64_t pipes[2] = {0xF0B16B00B5, 0xF0BADA5535}; //reading, writing

double batteryVoltage = 0.0;

void initRadio()
{
  radio.setPALevel(RF24_PA_HIGH);
  //payload size default 32...
  radio.setChannel(10); //we're team 10 :) 
  radio.setCRCLength(RF24_CRC_16); //2-byte CRC
  radio.setDataRate(RF24_1MBPS); //1Mbps data rate
  
  radio.openReadingPipe(0, pipes[0]); //reading pipe
  radio.openWritingPipe(pipes[1]);

  radio.startListening();

}
int const R_PIN = 6;
int const G_PIN = 7;
int const Y_PIN = 8;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  radio.begin();
  initRadio();

  Mouse.screenSize(1920, 1080);  // configure screen size
  
  randomSeed(analogRead(0));
  
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(Y_PIN, OUTPUT);
}

const double INPUT_MAX_VOLTS = 3.310;
const double R1 = .99;
const double R2 = 3.26;

const uint16_t CALIX = 0; //callibration for x
const uint16_t CALIY = 0;
const double scalingFactor = .002;
const uint16_t THRESHOLD = 500;

//Lights an LED for the specified time (in milliseconds) and then turns that LED Off
void lightLED(int pin,int timeToLight){
  digitalWrite(pin, HIGH);
  delay(timeToLight);
  digitalWrite(pin, LOW);
}

void turnOffLights(){
  digitalWrite(R_PIN, LOW);
  digitalWrite(G_PIN, LOW);
  digitalWrite(Y_PIN, LOW);
}


int moveVector[2] = {0, 0};

 void tiltToVector(uint16_t acceleration[]){
  moveVector[0] = 0;
  moveVector[1] = 0;
  if(abs(acceleration[0] - CALIX) > THRESHOLD){ //We should calculate move
    moveVector[0] = (int)(acceleration[0] * scalingFactor) - ;
  }
  if(abs(acceleration[1] - CALIY) > THRESHOLD) {
    moveVector[1] = (int)(acceleration[1] * scalingFactor) - ;
  }
}

struct data
{
  boolean isPushedR = false;
  boolean isPushedL = false;
  uint16_t batteryADC = 1.0;
  uint16_t acceleration[3] = {0, 0, 0};
  uint16_t gyroscope[3] = {0, 0, 0};
};

data packet;

void loop() {
  bool stillWaiting = true;
  Serial.println("About to read");
    while(stillWaiting){
      if(radio.available(0)){ //You've got mail!!!
        radio.read((char*) &packet, sizeof(packet));
        stillWaiting = false;
      }
    }
  Serial.println("Done Reading");
  //Turn on the appropriate battery led indicator
  turnOffLights();

  batteryVoltage = packet.batteryADC * (INPUT_MAX_VOLTS / 1023) * ((R1 + R2)/ R2);
  
  if(batteryVoltage > 3.9){
    digitalWrite(G_PIN, HIGH);
  } else if (batteryVoltage > 3.7){
    digitalWrite(Y_PIN, HIGH);
  } else {
    digitalWrite(R_PIN, HIGH);
  }
  Serial.println("Finished writing the pins");
  if (packet.isPushedR == true) {
    Serial.println("The right button has been clicked!!! (Did you mean to right click?!?!)");
  }
  if (packet.isPushedL == true) {
    Serial.println("The left button has been clicked!!! (Did you mean to left click?!?!)");
  }
  Serial.print("Voltage: ");
  Serial.println(batteryVoltage);
  Serial.println(packet.batteryADC);

  Serial.print("X: ");
  Serial.println(packet.acceleration[0]);
  
  Serial.print("Y: ");
  Serial.println(packet.acceleration[1]);
  
  Serial.print("Z: ");
  Serial.println(packet.acceleration[2]);

  tiltToVector(packet.acceleration);
  Mouse.move(moveVector[0], moveVector[1]);
  delay(25);
}
