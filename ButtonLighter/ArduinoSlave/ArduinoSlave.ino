void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

bool r, g , y = false;
void loop() {
<<<<<<< HEAD
  digitalWrite(10, r);
  digitalWrite(11, g);
  digitalWrite(12, y);
  
  if (Serial.avalable() > 0)
  {   
=======
  
    digitalWrite(10, r);
    digitalWrite(11, g);
    digitalWrite(12, y);
    
  if (Serial.avalable() > 0)
  {
>>>>>>> e51830769b5dc6b37841e46110d7053886544e0f
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
