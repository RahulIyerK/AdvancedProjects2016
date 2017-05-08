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
      desired = 0;
      error_sum = 0;
      pConst = pConstant;
      iConst = iConstant;
      dConst = dConstant;
      prev_error = 0;
      minMax = minMaxOfRange;
    }
    //returns the correction from the PID algorithm
    //make sure that you update the current and desired values so that the algorithm
    //reflects the most up-to-date data
    double getCorrection(){
      int error = desired - current;
      if(pastErrors.count() > maxSize){ //if we are about to go over...
        error_sum -= pastErrors.pop(); //then pop the queue
      }
      error_sum += error;
      pastErrors.push(error);
      int derivative = error - prev_error;
      prev_error = error;
      return PIDErrorToCorrection((pConst * error) + (iConst * error_sum) + (dConst * derivative));
    }

    //amount is a raw value (i.e. not a PID converted num)
    void setDesired(int rawAmount){
      desired = rawToPIDNum(rawAmount);
    }

    void setCurrent(int rawAmount) {
      current = rawToPIDNum(rawAmount);
    }

    //resets the PID constants to the values passed in
    void setConstants(double pConstant, double iConstant, double dConstant){
      pConst = pConstant;
      iConst = iConstant;
      dConst = dConstant;
    }

    int rawToPIDNum(double raw){
      return map(raw, -minMax, minMax, -PIDValueMinMax, PIDValueMinMax);
    }

    //returns a correction ranging from -1.0 to 1.0
    double PIDErrorToCorrection(double PIDError){
      double pidMinMax = (double)PIDValueMinMax;//typecast to a double to avoid those dreadful overflow problems
      double maxError = pidMinMax * 2;//get the min/max of pid
      double maxPIDError = maxError + (maxSize * maxError) + (maxError * 2); //calculate the total range that the error could be
      return (PIDError / maxPIDError);
    }

  private:
    int current; //the current position/angle
    int desired; //the desired position/angle
    int error_sum; //the total error within maxSize cycles
    double pConst; //proportional constant
    double iConst; //integrative constant
    double dConst; //derivative constant
    QueueList<int> pastErrors;
    const int maxSize = 1000;
    int prev_error;
    int minMax; //the max/min of the possible raw inputs
    //i.e. 30 if the possible inputs are from -30 to 30
    const int PIDValueMinMax = 100;
};

PIDAlgorithm pitchAlg = PIDAlgorithm(pitchPK, pitchIK, pitchDK, 30);
PIDAlgorithm rollAlg = PIDAlgorithm(rollPK, rollIK, rollDK, 30);
PIDAlgorithm yawAlg = PIDAlgorithm(yawPK, yawIK, yawDK, 30);

void setup() {
  
}


void loop() {
  
}
