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

long lastDebounceTime = 0;  // the last time the output pin was toggled
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
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
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
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
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
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
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
