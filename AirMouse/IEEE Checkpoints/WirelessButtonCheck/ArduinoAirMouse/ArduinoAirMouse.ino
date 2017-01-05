#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("started serial");
  
  pinMode(buttonPinR, INPUT);
  pinMode(buttonPinL, INPUT);

  radio.begin();
  initRadio();
  radio.stopListening();
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

    Serial.println("Starting to write");
    radio.write((char*) &packet, sizeof(packet));
    Serial.println(packet.batteryADC);
    Serial.println("Finished writing");
}
