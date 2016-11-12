#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"

RF24 radio(9,10);

uint64_t pipes[2] = {0xF0B16B00B5, 0xF0BADA5535}; //reading, writing

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


  
  randomSeed(analogRead(0));
  
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(Y_PIN, OUTPUT);
}

//Lights an LED for the specified time (in milliseconds) and then turns that LED Off
void lightLED(int pin,int timeToLight){
  digitalWrite(pin, HIGH);
  delay(timeToLight);
  digitalWrite(pin, LOW);
}

struct data
{
  boolean isPushedR = false;
  boolean isPushedL = false;
  double batteryVoltage = 0.0;
};

data game;

void loop() {
  //listen for response from arduino (p for pass, n for no pass)
  bool stillWaiting = true;
  char result;
  
  while(stillWaiting){
    
    if(radio.available(0)){ //You've got mail!!!
      radio.read((char*) &game, sizeof(game));
      stillWaiting = false;
    }
  }
  
  //if good, flash green led and continue otherwise flash red and reset
  if(game.batteryVoltage > 3.9){ //You've got a match or a lucky guess
    lightLED(G_PIN, 250);
  } else if (game.batteryVoltage > 3.7){
    lightLED(Y_PIN, 250);
  } else {
    lightLED(R_PIN, 250);
  }
  if (game.isPushedR == true) {
    Serial.println("The right button has been clicked!!! (Did you mean to right click?!?!)");
  }
  if (game.isPushedL == true) {
    Serial.println("The left button has been clicked!!! (Did you mean to left click?!?!)");
  }
}
