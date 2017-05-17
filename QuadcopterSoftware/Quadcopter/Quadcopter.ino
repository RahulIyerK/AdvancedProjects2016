//RF module includes
#include "Controller.h"
#include "RF24.h"
#include <SPI.h>

//IMU includes

//PID includes
#include <QueueList.h>

//RF module defines
#define channel 10 //our team number
#define PALevel RF24_PA_HIGH
#define CE A0
#define CS A1

//IMU defines

//Digital Output Pins
#define LED A3

//PWM Output Pins
#define MOTOR_1 5 
#define MOTOR_2 6
#define MOTOR_3 9
#define MOTOR_4 10 

//Analog Input Pins
#define VDIV_PIN A0 //voltage divider analog input pin

//Voltage Divider Constants

#define R1 1600 //"top" resistor in the voltage divider circuit
#define R2 1000 //"bottom" resistor in the voltage divider circuit
#define logicVoltage 3.3

//PID Constants

#define PITCH 0
#define ROLL 1
#define YAW 2
#define FACTOR 10

//PWM Constants
#define MAX_ADJUSTMENT 30 //the maximum adjustment to a base PWM
#define MIN_PWM 0 //the lowest the pwm is allowed to go
#define MAX_PWM 80 //the highest the pwm is allowed to go

//PID Global Variables

//don't make these constants as the controller will change them while tuning
double pitchPK = 0.5;
double pitchIK = 0.5;
double pitchDK = 0.5;

double rollPK = 0.5;
double rollIK = 0.5;
double rollDK = 0.5;

double yawPK = 0.5;
double yawIK = 0.5;
double yawDK = 0.5;

class PIDAlgorithm {
  public:
    PIDAlgorithm(double pConstant, double iConstant, double dConstant, int minMaxOfRange){
      PID_current = 0;
      PID_setpoint = 0;
      error_sum = 0;
      pConst = pConstant;
      iConst = iConstant;
      dConst = dConstant;
      prev_error = 0;
      minMax = minMaxOfRange;
      double pidMinMax = (double)PIDValueMinMax;//typecast to a double to avoid those dreadful overflow problems
      double maxError = pidMinMax * 2;//get the min/max of pid
      maxPIDError = (pConst * maxError) + (iConst * maxSize * maxError) + (dConst * maxError * 2); //calculate the total range that the error could be
    }
    
    //returns the correction from the PID algorithm
    //make sure that you update the current and setpoint values so that the algorithm
    //reflects the most up-to-date data
    double getCorrection(){
      int error = PID_setpoint - PID_current;
      if(pastErrors.count() > maxSize){ //if we are about to go over...
        error_sum -= pastErrors.pop(); //then pop the queue
      }
      error_sum += error;
      pastErrors.push(error);
      int derivative = error - prev_error;
      prev_error = error;
      return PIDErrorToCorrection((pConst * error) + (iConst * error_sum) + (dConst * derivative));
    }

    //the setpoint passed in is a raw setpoint value
    void setSetpoint(int rawAmount){
      PID_setpoint = rawToPIDNum(rawAmount); //the setpoint stored is converted to the PID range
    }

    //the current raw value that the system is at
    void setCurrent(int rawAmount) {
      PID_current = rawToPIDNum(rawAmount); //the current value stored is converted to the PID range
    }

    //resets the PID constants to the values passed in
    void setConstants(double pConstant, double iConstant, double dConstant){
      pConst = pConstant;
      iConst = iConstant;
      dConst = dConstant;
    }

    int rawToPIDNum(double raw){
      //return map(raw, -minMax, minMax, -PIDValueMinMax, PIDValueMinMax);
      return (int)(raw * FACTOR);
      //a PID number is the raw data (in the range [-minMax, minMax]) mapped to the range [-PIDValueMinMax, PIDValueMinMax]. 
    }

    //returns a correction ranging from -1.0 to 1.0
    //PIDError is the error from the PID algorithm ("PID Value")
    double PIDErrorToCorrection(double PIDError){
      return (PIDError / maxPIDError);
    }

  private:
    int PID_current; //the current value * FACTOR ("PID number")
    int PID_setpoint; //the setpoint value * FACTOR ("PID number")
    
    int32_t error_sum; //the total error within maxSize cycles..32 bit for large sum ("PID number")
    
    double pConst; //proportional constant for this object
    double iConst; //integrative constant for this object
    double dConst; //derivative constant for this object
    
    QueueList<int> pastErrors; //all the past error values ("PID number")
    
    const int maxSize = 1000; //max number of integers in the queue
    
    int prev_error; //used for derivative ("PID number")
    
    int minMax; //the max/min of the possible raw inputs (raw values)
    
    //i.e. 30 if the possible inputs are from -30 to 30
    const int PIDValueMinMax = minMax * FACTOR;

    double maxPIDError;
};

//Set up the PID Algorithms
PIDAlgorithm pitchAlg = PIDAlgorithm(pitchPK, pitchIK, pitchDK, 30);
PIDAlgorithm rollAlg = PIDAlgorithm(rollPK, rollIK, rollDK, 30);
PIDAlgorithm yawAlg = PIDAlgorithm(yawPK, yawIK, yawDK, 30);

RF24 radio(CE, CS);
rx_values_t rxValues; //re

// set up controller: pass it radio, channel #, and false since it is not the controller
Controller controller(&radio, channel, false);

//declare default speed variables
int baseSpeed; //the base PWM that the motors will be at

void setup() {
  Serial.begin(115200);
  // initalize the radio
  controller.init();
  pinMode(LED, OUTPUT);
  
}
void loop() {
  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNING CODE");
    return;
  }
  //only print values if new values have been received
  //controler.receive will return however many values were in the buffer
  if (controller.receive(&rxValues))
  {
    analogWrite(led, rxValues.throttle);
    Serial.print(" :\t"); Serial.print(rxValues.throttle);
    Serial.print("\t"); Serial.print(rxValues.yaw);
    Serial.print("\t"); Serial.print(rxValues.pitch);
    Serial.print("\t"); Serial.print(rxValues.roll);
    Serial.print("\t"); Serial.print(rxValues.flip);
    Serial.print("\t"); Serial.print(rxValues.highspeed);
    Serial.print("\t"); Serial.print(rxValues.P);
    Serial.print("\t"); Serial.print(rxValues.I);
    Serial.print("\t"); Serial.println(rxValues.D);
  }
  updateBattery();
  // send values for led back to the controller
  controller.send(&rxValues);

  //Get Setpoints and base speeds from controller
  int psp = 0; //TODO: replace these with the actual values from the controller
  int rsp = 0;
  int ysp = 0;
  baseSpeed = 0; //the base PWM that the motors will be at, this should be the throttle value
  
  //Get current angle values
  int pc = 0; //TODO: replace these with the actual values from the IMU
  int rc = 0; 
  int yc = 0;

  //Do PID
  //first update the algorithms with the new data
  pitchAlg.setCurrent(pc);
  pitchAlg.setSetpoint(ysp);
  rollAlg.setCurrent(rc);
  rollAlg.setSetpoint(rsp);
  yawAlg.setCurrent(yc);
  yawAlg.setSetpoint(ysp);
  //Now calculate the errors on a scale from -1.0 to 1.0
  double pitchCorrection = pitchAlg.getCorrection();
  double rollCorrection = rollAlg.getCorrection();
  double yawCorrection = yawAlg.getCorrection();

  //Now make the corrections to the motors
  //TODO:
  //For each correction, add or subtract each
  //motor PWM by the (correction * MAX_ADJUSTMENT)
  //this will look like the following
  //but for each motor AND for each correction (since the corrections will compound)
  
  //first set all the motors to the base speed
  //i.e. motor1 = baseSpeed;
  //now do all of the corrections for each motor for each dimension
  //i.e. motor1 += (correction * MAX_ADJUSTMENT);
  for(int j = 0; j < 3; j++){
    if(j == PITCH) {
      //do pitch corrections for each motor
    } else if (j == ROLL) {
      //do roll corrections for each motor
    } else { //must be YAW
      //do yaw corrections for each motor
    }
  }
  //now constrain the values of the PWM so that we don't go too high or too low
  //i.e. motor1 = constrain(motor1, MIN_PWM, MAX_PWM);
}
void updateBattery() {
  int batRead = analogRead(contBattPin);
  //calc bat voltage (mV)
  unsigned long batVolt = (batRead * logicVolt * (R3 + R4)) / (R4) * 1000 / 1023;
  if (batVolt < 7400) {
    // tell the controller to turn on the led
    rxValues.auxLED = true;
  }
}
