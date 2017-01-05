void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode (10, OUTPUT);
  pinMode (11, OUTPUT);
  pinMode (12, OUTPUT);
}
bool r, g, y = false;
void loop() {
   digitalWrite(10, r);
   digitalWrite(11, g);
   digitalWrite(12, y);
      
  if(Serial.available() > 0){
     
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


