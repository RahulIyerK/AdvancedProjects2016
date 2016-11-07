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
  
  radio.openReadingPipe(0, pipes[0]); //reading pipe
  radio.openWritingPipe(pipes[1]);
}

const int buttonPinR = 2;
const int buttonPinG = 4;
const int buttonPinY = 6;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  
  pinMode(buttonPinR, INPUT);
  pinMode(buttonPinG, INPUT);
  pinMode(buttonPinY, INPUT);

  radio.startListening();
}

long lastDebounceTimeR = 0;  // the last time the output pin was toggled
long lastDebounceTimeG = 0;
long lastDebounceTimeY = 0;
long debounceDelay = 50;
int buttonStateR = LOW;             // the current reading from the input pin
int buttonStateG = LOW;
int buttonStateY = LOW;
int lastReadingR = LOW;
int lastReadingG = LOW;
int lastReadingY = LOW;


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

char readButtonG(){
  int reading = digitalRead(buttonPinG);
  char out = 'a';
  
  if (reading != lastReadingG) {
    // reset the debouncing timer
    lastDebounceTimeG = millis();
  }

  if ((millis() - lastDebounceTimeG) > debounceDelay) {
    
    if (reading != buttonStateG)
    {
      out = 'g';
    }
    buttonStateG = reading;
  }

  lastReadingG = reading;
  return out;
}

char readButtonY(){
  int reading = digitalRead(buttonPinY);
  char out = 'a';
  
  if (reading != lastReadingY) {
    // reset the debouncing timer
    lastDebounceTimeY = millis();
  }

  if ((millis() - lastDebounceTimeY) > debounceDelay) {
    
    if (reading != buttonStateY)
    {
      out = 'y';
    }
    buttonStateY = reading;
  }

  lastReadingY = reading;
  return out;
}

char sequenceArray [100];
int roundNum = 1;

struct data
{
  char sequenceArraySend [25];
  char result = 'p';
};

data game;

void loop() {
  //reads character sequence for that round
  //checks button sequence
  //sends back 'p'/'n'
  int counter = 0;
  while(counter < 4) {
    if(radio.available(0)){
      radio.read((char*) &game, sizeof(game));
      for (int j = 0; j < 25; j++){    
      sequenceArray[(counter * 25) + j] = game.sequenceArraySend[j];
      }
      counter++;
    }
  }
    
    
    int countPresses = 0;

    char userAnswer[roundNum];
    
    while (countPresses < roundNum)
    {
      
      if(readButtonR() == 'r' && buttonStateR == HIGH)
      {
          userAnswer[countPresses] = 'r';
          countPresses++;
      }
      if(readButtonG() == 'g' && buttonStateG == HIGH)
      {
          userAnswer[countPresses] = 'g';
          countPresses++;
      }
      if(readButtonY() == 'y' && buttonStateY == HIGH)
      {
        userAnswer[countPresses] = 'y';
        countPresses++;
      }
    }
    
    bool answerCorrect = true;
    for (int i = 0; i<roundNum; i++)
    {
      if (sequenceArray[i] != userAnswer[i])
      {
        answerCorrect = false;
      }
    }
    
    if (answerCorrect)
    {
      game.result = 'p';
      roundNum++;
    }
    else
    {
      game.result = 'n';
      roundNum = 1;
    }
    radio.stopListening();
    
    radio.write((char*) &game, sizeof(game));
    radio.startListening();
  
}
