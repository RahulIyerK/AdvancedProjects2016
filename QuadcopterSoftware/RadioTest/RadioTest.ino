#include "Controller.h" //cullenmq's nRF24L01+ controller library
#include "printf.h"
#include <SPI.h>
#define channel 10 //we're team 10
#define PALevel RF24_PA_HIGH //Power Level
#define CE A0 //the CE connection on our quadcopter board
#define CS A1 //the CSN connection on our quadcopter board

RF24 radio(CE, CS); //initialize the nRF24L01+ radio module
 
rx_values_t rxValues; 

// set up controller: pass it radio, channel #, and false since it is not the controller
Controller controller(&radio, channel, false);


void setup() {
  Serial.begin(38400);

  controller.init(); //initialize radio and connection with controller

}

void loop() {
  //from example code in Controller library
  
  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNING CODE");
    return;
  }
  
  //only print values if new values have been received
  //controller.receive will return however many values were in the buffer
  
  if (controller.receive(&rxValues))
  {
    Serial.print(" :\t"); Serial.print(rxValues.throttle);
    Serial.print("\t"); Serial.print(rxValues.yaw);
    Serial.print("\t"); Serial.print(rxValues.pitch);
    Serial.print("\t"); Serial.print(rxValues.roll);
    Serial.print("\t"); Serial.print(rxValues.flip);
    Serial.print("\t"); Serial.print(rxValues.highspeed);
    Serial.print("\t"); Serial.print(rxValues.P);
    Serial.print("\t"); Serial.print(rxValues.I);
    Serial.print("\t"); Serial.println(rxValues.D);
  }
}
