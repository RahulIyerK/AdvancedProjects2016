const int buttonPinR = 2;
const int buttonPinG = 4;
const int buttonPinY = 6;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
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

void loop() {
    if(readButtonR() == 'r'){
      Serial1.write('r');
    }
    if(readButtonG() == 'g'){
      Serial1.write('g');
    }
    if(readButtonY() == 'y'){
      Serial1.write('y');
    }
}
