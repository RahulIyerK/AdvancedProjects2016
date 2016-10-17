void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

char getNewLight(){
  int random = rand() * 3;
  if(random == 0){
    return 'r';
  } else if(random == 1){
    return 'g';
  } else {
    return 'y';
  }
  
}

bool r, g , y = false;
int round = 1;
String sequence = "";

void loop() {

  //add on to sequence
  //light up leds for the round
  //listen for response from arduino
  //if good, flash green led and continue otherwise flash an abort sequence
 
  if (Serial.available() > 0)
  {

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
