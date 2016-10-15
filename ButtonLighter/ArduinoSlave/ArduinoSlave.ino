void setup() {
  Serial.begin(9600);

}

bool r, g , y = false;
void loop() {
  if (Serial.avalable() > 0)
  {
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

}
