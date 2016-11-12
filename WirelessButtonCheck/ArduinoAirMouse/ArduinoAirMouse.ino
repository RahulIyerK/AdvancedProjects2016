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
  
  radio.openWritingPipe(pipes[1]);
}

const int buttonPinR = 3;
const int buttonPinL = 5;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("started serial");
  
  pinMode(buttonPinR, INPUT);
  pinMode(buttonPinL, INPUT);

  radio.begin();
  initRadio();
}

long lastDebounceTimeR = 0;  // the last time the output pin was toggled
long lastDebounceTimeL = 0;
long debounceDelay = 50;
int buttonStateR = LOW;             // the current reading from the input pin
int buttonStateL = LOW;
int lastReadingR = LOW;
int lastReadingL = LOW;

const double VOLTAGE_ADC_RATIO = (3.2232 / 1023) * ((3.3 + 1)/3.3);

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
  double batteryVoltage = 0.0;
};

data packet;

void loop() {    
    if (readButtonR() == 'r') {
      packet.isPushedR = true;
    }
    else {
      packet.isPushedR = false;
    }
    if (readButtonL() == 'l') {
      packet.isPushedL = true;
    }
    else {
      packet.isPushedL = false;
    }
    Serial.println("Finished reading in presses");

    packet.batteryVoltage = (analogRead(1) * VOLTAGE_ADC_RATIO);
    
    radio.write((char*) &packet, sizeof(packet));
}
