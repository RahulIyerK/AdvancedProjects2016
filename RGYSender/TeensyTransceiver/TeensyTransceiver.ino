void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  if(Serial.available() > 0){
    char rgy = Serial.read();
    if(rgy == 'r' || rgy == 'g' || rgy == 'y'){
      Serial1.write(rgy);
    }
  }

}
