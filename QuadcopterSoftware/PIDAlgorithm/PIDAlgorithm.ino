#include <QueueList.h>

#define PITCH 0
#define ROLL 1
#define YAW 2

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
      current = 0;
      setpoint = 0;
      error_sum = 0;
      pConst = pConstant;
      iConst = iConstant;
      dConst = dConstant;
      prev_error = 0;
      minMax = minMaxOfRange;
    }
    
    //returns the correction from the PID algorithm
    //make sure that you update the current and setpoint values so that the algorithm
    //reflects the most up-to-date data
    double getCorrection(){
      int error = setpoint - current;
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
      setpoint = rawToPIDNum(rawAmount); //the setpoint stored is converted to the PID range
    }

    //the current raw value that the system is at
    void setCurrent(int rawAmount) {
      current = rawToPIDNum(rawAmount); //the current value stored is converted to the PID range
    }

    //resets the PID constants to the values passed in
    void setConstants(double pConstant, double iConstant, double dConstant){
      pConst = pConstant;
      iConst = iConstant;
      dConst = dConstant;
    }

    int rawToPIDNum(double raw){
      return map(raw, -minMax, minMax, -PIDValueMinMax, PIDValueMinMax);
      //a PID number is the raw data (in the range [-minMax, minMax]) mapped to the range [-PIDValueMinMax, PIDValueMinMax]. 
    }

    //returns a correction ranging from -1.0 to 1.0
    double PIDErrorToCorrection(double PIDError){
      double pidMinMax = (double)PIDValueMinMax;//typecast to a double to avoid those dreadful overflow problems
      double maxError = pidMinMax * 2;//get the min/max of pid
      double maxPIDError = maxError + (maxSize * maxError) + (maxError * 2); //calculate the total range that the error could be
      return (PIDError / maxPIDError);
    }

  private:
    int PID_current; //the current value * 100 ("PID number")
    int PID_setpoint; //the setpoint value * 100 ("PID number")
    
    int32_t error_sum; //the total error within maxSize cycles..32 bit for large sum ("PID number")
    
    double pConst; //proportional constant for this object
    double iConst; //integrative constant for this object
    double dConst; //derivative constant for this object
    
    QueueList<int> pastErrors; //all the past error values ("PID number")
    
    const int maxSize = 1000; //max number of integers in the queue
    
    int prev_error; //used for derivative ("PID number")
    
    int minMax; //the max/min of the possible raw inputs (raw values)
    
    //i.e. 30 if the possible inputs are from -30 to 30
    const int PIDValueMinMax = minMax * 100;
};

PIDAlgorithm pitchAlg = PIDAlgorithm(pitchPK, pitchIK, pitchDK, 30);
PIDAlgorithm rollAlg = PIDAlgorithm(rollPK, rollIK, rollDK, 30);
PIDAlgorithm yawAlg = PIDAlgorithm(yawPK, yawIK, yawDK, 30);

void setup() {
  
}


void loop() {
  
}
