#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "printf.h"


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

  uint8_t myInt;


void setup() {
  printf_begin();
  Serial.begin(9600);
  // put your setup code here, to run once:
  radio.begin();
  initRadio();

  
  radio.printDetails();
 
}



void loop() {

}
