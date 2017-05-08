#define motor1 1 //motor1 connection to atmega on quadcopter board
#define motor2 2 //motor2 connection to atmega on quadcopter board
#define motor3 3 //motor3 connection to atmega on quadcopter board
#define motor4 4 //motor4 connection to atmega on quadcopter board

void setup() {
  Serial.begin(38400);
  
  pinMode (motor1, OUTPUT);
  pinMode (motor2, OUTPUT);
  pinMode (motor3, OUTPUT);
  pinMode (motor4, OUTPUT);

}

void loop() {
  
  for (int i = 0; i <= 255; i++)
  {
    Serial.println(i);
    
    analogWrite(motor1, i);
    analogWrite(motor2, i);
    analogWrite(motor3, i);
    analogWrite(motor4, i);
    
    delay(100); //make the motor speed changes occur at a slower rate
  }
  for (int i = 255; i >= 0; i--)
  {
    Serial.println(i);
    
    analogWrite(motor1, i);
    analogWrite(motor2, i);
    analogWrite(motor3, i);
    analogWrite(motor4, i);
    `
    delay(100); //make the motor speed changes occur at a slower rate
  }

}
