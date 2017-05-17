//RF module includes
#include "Controller.h"
#include "RF24.h"
#include <SPI.h>

//IMU includes


//RF module defines
#define channel 10 //our team number
#define PALevel RF24_PA_HIGH
#define CE A0
#define CS A1

//IMU defines

//Digital Output Pins
#define LED A3

//PWM Output Pins
#define MOTOR_1 5 
#define MOTOR_2 6
#define MOTOR_3 9
#define MOTOR_4 10 

//Analog Input Pins
#define VDIV_PIN A0 //voltage divider analog input pin

//Voltage Divider Constants

#define R1 1600 //"top" resistor in the voltage divider circuit
#define R2 1000 //"bottom" resistor in the voltage divider circuit
#define logicVoltage 3.3



RF24 radio(CE, CS);
rx_values_t rxValues; //re

// set up controller: pass it radio, channel #, and false since it is not the controller
Controller controller(&radio, channel, false);


void setup() {
  Serial.begin(115200);
  // initalize the radio
  controller.init();
  pinMode(LED, OUTPUT);
  
}
void loop() {
  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNING CODE");
    return;
  }
  //only print values if new values have been received
  //controler.receive will return however many values were in the buffer
  if (controller.receive(&rxValues))
  {
    analogWrite(led, rxValues.throttle);
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
  updateBattery();
  // send values for led back to the controller
  controller.send(&rxValues);
}
void updateBattery() {
  int batRead = analogRead(contBattPin);
  //calc bat voltage (mV)
  unsigned long batVolt = (batRead * logicVolt * (R3 + R4)) / (R4) * 1000 / 1023;
  if (batVolt < 7400) {
    // tell the controller to turn on the led
    rxValues.auxLED = true;
  }
}
