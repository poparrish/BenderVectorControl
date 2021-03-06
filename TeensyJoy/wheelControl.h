#include "returnedVariables.h"
class wheelControl {
  public:
  float Pgain;
  float Dgain;
  int pulsesPerRot;
  float RPMerror;
  float DerRPMerror;
  float lastCycleSpeedCheck;
  float speedCheckFloat;
  float lastcycleRPM;
  float curRPM;
  float lastDesiredRPM;
  float desiredHubRPM;
  float RPMincrement;
  float desiredRPM;
  int speedCheck;
  int speedWrite;
  bool forwardBackward;
  bool backToZero;
  int cutoff;
  float hubPerror, hubIerror, hubDerror;
  float hubKp, hubKi, hubKd;
  float lastHubPerror;
  float lastSpeedCheckFloat;
  
  // inits for planetary motor

  float Perror;
  float Ierror;
  float Derror;
  float lastPerror;
  float deltaEncPos;
  float cyclesPerRot;
  float RPMPlanMot;
  int lastEncPos;
  float currentPlanMotPos;
  float lastPlanMotPos;
  float deltaPlanMotPos;
  float derPlanMotPos;
  float motorSpeed;
  int intmotorSpeed;
  float PWheelGain;
  float DWheelGain;
  bool PlanMotDir;
  int Hz;

returnVariables calculate(float a, float b, float c, unsigned long d, int e);
wheelControl(); 
};

