#include <SoftwareSerial.h>

const int buttonPinR = 2;
const int buttonPinG = 4;
const int buttonPinY = 6;

SoftwareSerial srl (8,9);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  srl.begin(9600);
  
  pinMode(buttonPinR, INPUT);
  pinMode(buttonPinG, INPUT);
  pinMode(buttonPinY, INPUT);
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

int roundNum = 1;

void loop() {
  //reads character sequence for that round
  //checks button sequence
  //sends back 'p'/'n'
  if (srl.available()>0)
  {
    Serial.print("received: ");
    char sequence [roundNum];
    
    srl.readBytesUntil('\n', sequence, roundNum);
    
    Serial.print(roundNum);
    Serial.print(" characters:");
    Serial.println(sequence);
    
    
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
      if (sequence[i] != userAnswer[i])
      {
        answerCorrect = false;
      }
    }
    
    if (answerCorrect)
    {
      srl.write('p');
      roundNum++;
    }
    else
    {
      srl.write('n');
      roundNum = 1;
    }
    
  }
  
  
}
