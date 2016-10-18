#include <SoftwareSerial.h>
int const R_PIN = 3;
int const G_PIN = 6;
int const Y_PIN = 9;

SoftwareSerial srl(11, 12);

void setup() {
  srl.begin(9600);
  randomSeed(analogRead(0));
  Serial.begin(9600);
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

String sequenceString = "";

void loop() {

  //add on to sequence
  sequenceString = sequenceString + getNewLight();
  Serial.println(sequenceString);
  
  int sequenceLength = sizeof(sequenceString);
  
  char sequenceCharArray [sequenceLength];
  
  for (int i = 0; i< sequenceLength; i++)
  {
    sequenceCharArray[i] = sequenceString.charAt(i);
  }
  
  
  //light up leds for the round
  for(int j = 0; j < sequenceLength; j++){
    delay(250);
    if(sequenceCharArray[j] == 'r'){
      lightLED(R_PIN, 1000);
    } else if(sequenceCharArray[j] == 'g'){
      lightLED(G_PIN, 1000);
    } else if (sequenceCharArray[j] == 'y'){
      lightLED(Y_PIN, 1000);
    }
  }

  //write over softwareserial
  srl.write(sequenceCharArray);
  
  //listen for response from arduino (p for pass, n for no pass)
  bool stillWaiting = true;
  char result;
  
  while(stillWaiting){
    
    if(srl.available() > 0){ //You've got mail!!!
      result = srl.read();
      stillWaiting = false;
    }
  }
  
  //if good, flash green led and continue otherwise flash red and reset
  if(result == 'p'){ //You've got a match or a lucky guess
    for(int k = 0; k < 3; k++){
      lightLED(G_PIN, 250);
      delay(250);//No need to reset sequence as they passed the test
    }
  } else {
    for(int k = 0; k < 3; k++){
      lightLED(R_PIN, 250);
      delay(250);
    }
    sequenceString = "";
  }
  delay(2500);//give time for players to realize the result and then prepare to memorize

}
