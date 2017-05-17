#include "Controller.h"
#include "printf.h"

//THIS IS YOUR TEAM NUMBER
#define channel 10
#define PALevel RF24_PA_HIGH
#define CE A0
#define CS A1
// test led (should be pwm)
#define led 3
/********************************
 **** Voltage Divider Consts ****
 ********************************/
#define contBattPin A2
#define R4 1600
#define R3 1000
#define logicVolt 3.3

RF24 radio(CE, CS);
rx_values_t rxValues;
// set up controller: pass it radio, channel #, and false since it is not the controller
Controller controller(&radio, channel, false);
void setup() {
  Serial.begin(38400);
  // initalize the radio
  controller.init();
  pinMode(led, OUTPUT);
}
void loop() {
  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNING CODE");
    return;
  }
  //only print values if new values have been received
  //controller.receive will return however many values were in the buffer
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
