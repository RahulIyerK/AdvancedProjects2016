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
int buttonStateR;             // the current reading from the input pin
int buttonStateG;
int buttonStateY;
int lastButtonStateR = LOW;
int lastButtonStateG = LOW;
int lastButtonStateY = LOW;

char readButtonR(){
  int reading = digitalRead(buttonPinR);
  char out = 'a';
  
  if (reading != lastButtonStateR) {
    // reset the debouncing timer
    lastDebounceTimeR = millis();
  }

  if ((millis() - lastDebounceTimeR) > debounceDelay) {
    buttonStateR = reading;
    out = 'r';
  }

  lastButtonStateR = buttonStateR;
  return out;
}

char readButtonG(){
  int reading = digitalRead(buttonPinG);
  char out = 'a';
  
  if (reading != lastButtonStateG) {
    // reset the debouncing timer
    lastDebounceTimeG = millis();
  }

  if ((millis() - lastDebounceTimeG) > debounceDelay) {
    buttonStateG = reading;
    out = 'g';
  }

  lastButtonStateG = buttonStateG;
  return out;
}

char readButtonY(){
  int reading = digitalRead(buttonPinY);
  char out = 'a';
  
  if (reading != lastButtonStateY) {
    // reset the debouncing timer
    lastDebounceTimeY = millis();
  }

  if ((millis() - lastDebounceTimeY) > debounceDelay) {
    buttonStateY = reading;
    out = 'y';
  }

  lastButtonStateY = buttonStateY;
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
