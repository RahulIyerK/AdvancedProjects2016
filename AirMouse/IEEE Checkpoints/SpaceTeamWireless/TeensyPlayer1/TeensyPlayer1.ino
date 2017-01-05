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

//returns a random char (r, g, or y)
char getNewLight(){
  long randomNum = random(0,3);
  if(randomNum == 0){
    return 'r';
  } else if(randomNum == 1){
    return 'g';
  } else {
    return 'y';
  } 
}

//Lights an LED for the specified time (in milliseconds) and then turns that LED Off
void lightLED(int pin,int timeToLight){
  digitalWrite(pin, HIGH);
  delay(timeToLight);
  digitalWrite(pin, LOW);
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

  //add on to sequence
  sequenceArray [roundNum - 1] = getNewLight();
  Serial.println(sequenceArray);
  
  
  //light up leds for the round
  for(int j = 0; j < roundNum; j++){
    delay(250);
    if(sequenceArray[j] == 'r'){
      lightLED(R_PIN, 1000);
    } else if(sequenceArray[j] == 'g'){
      lightLED(G_PIN, 1000);
    } else if (sequenceArray[j] == 'y'){
      lightLED(Y_PIN, 1000);
    }
  }

  radio.stopListening();
  //write over radio
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 25; j++)
    {
      game.sequenceArraySend[j] = sequenceArray[(i * 25) + j];
    }
    radio.write((char*) &game, sizeof(game));
  }

  radio.startListening();
  
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
  if(game.result == 'p'){ //You've got a match or a lucky guess
    for(int k = 0; k < 3; k++){
      lightLED(G_PIN, 250);
      delay(250);//No need to reset sequence as they passed the test
    }
    roundNum++;
  } else {
    for(int k = 0; k < 3; k++){
      lightLED(R_PIN, 250);
      delay(250);
    }
    roundNum = 1;
  }
  delay(1000);//give time for players to realize the result and then prepare to memorize

}
