#define readPin A2 //atmega voltage divider output connection on quadcopter board

#define R1 1000 //the resistor after which the voltage is read in the voltage divider
#define R2 1600 //voltage divider resistor across which the voltage is read

#define logicHIGH 3.3 //the high voltage of our Arduino 

#define LED A3 //atmega LED connection on quadcopter board

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(dividerPin, INPUT);
}

void loop() {
   int batteryReadValue = analogRead(readPin);
  //calc battery voltage (in milliVolts)
  unsigned long batteryVoltage = (batteryReadValue * logicHIGH * (R1 + R2)) / (R2) * 1000 / 1023;
  
  if (batteryVoltage < 7400) //7400mV = 7.4V
  {
    digitalWrite(LED, HIGH); //we turn on the LED when the batteryVoltage is below nominal
    
  }
}
