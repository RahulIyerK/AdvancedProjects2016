//RF module includes
#include "Controller.h"
#include "RF24.h"
#include <SPI.h>

//IMU includes

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

//IMU status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void initIMU()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        dmpReady = true;
        
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        Serial.println("Hello");
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


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
#define MOTOR_FL 5 
#define MOTOR_FR 6
#define MOTOR_BR 9
#define MOTOR_BL 10 

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


/*
 * Yaw:..
 * 
 * Pitch: Actually Roll reported by MPU, sign is backwards
 * Roll: Actually Pitch...sign is correct
 */


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
int throttle; //the base PWM that the motors will be at

void setup() {
  Serial.begin(115200);
  // initalize the radio
  controller.init();
  pinMode(LED, OUTPUT);
  initIMU();

  Serial.println("asdfasdf");
  
}
void loop() {
  //Serial.println("stuck1");
  if (!dmpReady) 
  {
    Serial.println("DMP NOT READY");
    return;
  }
  //Serial.println("stuck2");


    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    //Serial.println("stuck3");
    
    // check for overflow (this should never happen unless our code is too inefficient)
    
    //Serial.print("fifo Count: ");
    //Serial.println(fifoCount);
    if (fifoCount == 1024) {
      //Serial.println("stuck4");
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println("stuck5");
        Serial.println(F("FIFO overflow!"));
    } else {
        // wait for correct available data length, should be a VERY short wait
        //Serial.println("stuck6");
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        //Serial.println("stuck7");
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //Serial.println("stuck8");
       
        
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);

        //Serial.println("stuck9");
    }

  //Serial.println("after mpu stuff");
  
  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNNING CODE");
    return;
  }

  //Serial.println("controller is functioning");
  //only print values if new values have been received
  //controler.receive will return however many values were in the buffer

  //Serial.println(controller.receive(&rxValues));
  
  if (controller.receive(&rxValues))
  {
    
    Serial.print(" :\t"); Serial.print(rxValues.throttle);
    
    Serial.print("\t"); Serial.print(rxValues.yaw);
    Serial.print("\t"); Serial.print(rxValues.pitch);
    Serial.print("\t"); Serial.print(rxValues.roll);
    
    Serial.print("\t"); Serial.print(rxValues.flip);
    Serial.print("\t"); Serial.print(rxValues.highspeed);
    
    Serial.print("\t"); Serial.print(rxValues.P);
    Serial.print("\t"); Serial.print(rxValues.I);
    Serial.print("\t"); Serial.println(rxValues.D);
    
    updateBattery();
    
    // send values for led back to the controller
    controller.send(&rxValues);
  
    //Get Setpoints and base speeds from controller
    int rsp = rxValues.pitch; //roll is actually pitch
    int psp = rxValues.roll; //pitch is actually roll
    
    int ysp = rxValues.yaw;
    throttle = rxValues.throttle; //the base PWM that the motors will be at, this should be the throttle value
    
    //Get current angle values
    int pc = ypr[2]; //actually roll
    int rc = ypr[1]; //actually pitch
    int yc = ypr[0]; 
  
    //Do PID
    //first update the algorithms with the new data
    pitchAlg.setCurrent(pc);
    pitchAlg.setSetpoint(ysp);
    rollAlg.setCurrent(rc);
    rollAlg.setSetpoint(rsp);
    yawAlg.setCurrent(yc);
    yawAlg.setSetpoint(ysp);
    
    //Now calculate the errors on a scale from -1.0 to 1.0
    double pitchCorrection = pitchAlg.getCorrection() * MAX_ADJUSTMENT;
    double rollCorrection = rollAlg.getCorrection() * MAX_ADJUSTMENT;
    double yawCorrection = yawAlg.getCorrection() * MAX_ADJUSTMENT;
  
    
    //first set all the motors to the base speed
    //i.e. motor1 = baseSpeed;
    int motorfl = throttle;
    int motorfr = throttle;
    int motorbr = throttle;
    int motorbl = throttle;
    
    //now do all of the corrections for each motor for each dimension
    //i.e. motor1 += (correction * MAX_ADJUSTMENT);
    
    for(int j = 0; j < 3; j++){
      if(j == PITCH) {
        //do pitch corrections for each motor
        motorfl += pitchCorrection;
        motorfr += pitchCorrection;
        motorbr -= pitchCorrection;
        motorbl -= pitchCorrection;
      } else if (j == ROLL) {
        //do roll corrections for each motor
        motorfl += rollCorrection;
        motorfr -= rollCorrection;
        motorbr -= rollCorrection;
        motorbl += rollCorrection;
      } else { //must be YAW
        //do yaw corrections for each motor
        motorfl += yawCorrection;
        motorfr -= yawCorrection;
        motorbr += yawCorrection;
        motorbl -= yawCorrection;
      }
    }
    //now constrain the values of the PWM so that we don't go too high or too low
    //i.e. motor1 = constrain(motor1, MIN_PWM, MAX_PWM);
    
    analogWrite(MOTOR_FL, constrain(motorfl, MIN_PWM, MAX_PWM));
    analogWrite(MOTOR_FR, constrain(motorfr, MIN_PWM, MAX_PWM));
    analogWrite(MOTOR_BR, constrain(motorbr, MIN_PWM, MAX_PWM));
    analogWrite(MOTOR_BL, constrain(motorbl, MIN_PWM, MAX_PWM));
  }
  else
  {
    //Serial.println("did not receive values");
  }
}


void updateBattery() {
//  int batRead = analogRead(contBattPin);
//  //calc bat voltage (mV)
//  unsigned long batVolt = (batRead * logicVolt * (R3 + R4)) / (R4) * 1000 / 1023;
//  if (batVolt < 7400) {
//    // tell the controller to turn on the led
//    rxValues.auxLED = true;
//  }


}


