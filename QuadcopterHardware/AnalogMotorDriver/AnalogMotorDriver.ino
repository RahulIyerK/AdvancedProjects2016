const int analogOutPin = 9; // Analog output pin that the LED is attached to

int maxValue = 220;
int minValue = 110;
double multiplier = .70;

void setup() {
  // initialize serial communications at 9600 bps:
  pinMode(analogOutPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  /*analogWrite(analogOutPin,maxValue * multiplier);
  Serial.println(multiplier);
*/
  for (int i = minValue; i < maxValue;i++){
    analogWrite(analogOutPin,i);
    delay(5);
  }

  for (int i = maxValue; i > minValue;i--){
    analogWrite(analogOutPin,i);
    delay(5);
  }
  
}
