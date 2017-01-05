void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  if(Serial.available() > 0){
    Serial1.write(Serial.read());
  }

}
