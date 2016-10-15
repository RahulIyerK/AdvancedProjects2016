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
  char ouptput = 'a';
  
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    buttonStateR = reading;
    output = 'r';
  }

  lastButtonStateR = buttonStateR;
  return output;
}

char readButtonG(){
  int reading = digitalRead(buttonPinG);
  char ouptput = 'a';
  
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    buttonStateG = reading;
    output = 'g';
  }

  lastButtonStateG = buttonStateG;
  return output;
}

char readButtonY(){
  int reading = digitalRead(buttonPinY);
  char ouptput = 'a';
  
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    buttonStateY = reading;
    output = 'y';
  }

  lastButtonStateY = buttonStateY;
  return output;
}

void loop() {
    char rgy = 
    if(rgy == 'r' || rgy == 'g' || rgy == 'y'){
      Serial1.write(rgy);
    }
  

}
