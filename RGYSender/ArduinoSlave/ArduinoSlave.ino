void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
bool r, g, y = false;
void loop() {
  if(Serial.available() > 0){
      digitalWrite(10, r);
      digitalWrite(11, g);
      digitalWrite(12, y);
      
    char rgy = Serial.read();
    if(rgy == 'r'){
      r = !r;
    } else if(rgy == 'g'){
      g = !g;
    } else {
      y = !y;
    }
    }
  }


