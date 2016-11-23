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
#define R_PIN 6 //Red LED
#define G_PIN 7 //Green LED
#define Y_PIN 8 //Yellow LED

void setup() {
  Serial.begin(9600);
  while(!Serial); //wait until Serial is initialized...(we found that not including this line of code caused errors on the 
                  //Teensy because it started executing code without ensuring that Serial communication with the laptop was
                  //properly initialized...
  radio.begin();
  initRadio();

  Mouse.screenSize(1920, 1080);  // configure screen size
  
  randomSeed(analogRead(0));
  
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(Y_PIN, OUTPUT);
}

#define INPUT_MAX_VOLTS 3.310 //MEASURED Vcc of Arduino Pro Mini when powered by battery (this number must be changed when 
                              //the Arduino is powered by a laptop
#define R1 .99 //"top" resistor in voltage divider
#define R2 3.26 //"bottom" resistor in voltage divider


#define CALIX 0 //calibration for X
#define CALIY -225 //calibration for Y

#define YMIN -2630 //unused
#define YMAX 3290 //unused

#define XMIN -3600 //unused
#define XMAX 3900 //unused


#define scalingFactor .007
#define THRESHOLD 500

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

void tiltToVector(const uint16_t acceleration[]){
  moveVector[0] = 0;
  moveVector[1] = 0;
  if(abs((int16_t)acceleration[0] - CALIX) > THRESHOLD){ //calculate move
    moveVector[1] = (int)((int16_t)acceleration[0] * scalingFactor);
  }
  if(abs((int16_t)acceleration[1] - CALIY) > THRESHOLD) {
    moveVector[0] = (int)((int16_t)acceleration[1] * scalingFactor);
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
  
  Mouse.move(moveVector[0], moveVector[1]); //added for smoothness

  
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
  
  Mouse.move(moveVector[0], moveVector[1]); //added for smoothness

  //prints for debugging purposes
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
  Serial.println((int16_t) packet.acceleration[0]);
  
  Serial.print("Y: ");
  Serial.println((int16_t) packet.acceleration[1]);
  
  Serial.print("Z: ");
  Serial.println((int16_t) packet.acceleration[2]);

  tiltToVector(packet.acceleration); //re-calculate move vector coordinates
  
  //todo: implement scrolling and other cool mouse functions!
  if (packet.isPushedL && packet.isPushedR)
  {
    Mouse.set_buttons(0,1,0);
  }
  else {
    Mouse.set_buttons(packet.isPushedL, 0, packet.isPushedR);
  }
  
  Mouse.move(moveVector[0], moveVector[1]);
  
  //delay(5);
}
