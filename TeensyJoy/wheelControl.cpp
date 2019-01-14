#include <Arduino.h>
//#include <PID_v1.h>
#include "wheelControl.h"



wheelControl::wheelControl(){
  Hz = 30;
  pulsesPerRot = 45;
  RPMerror = 0;
  DerRPMerror = 0;
  lastCycleSpeedCheck = 0;
  speedCheckFloat = 0;
  lastSpeedCheckFloat = 0;
  lastcycleRPM = 0;
  lastDesiredRPM = 0;
  desiredHubRPM = 0;
  RPMincrement = 0;
  speedCheck = 0;
  
  forwardBackward = false;
  backToZero = false;
  cyclesPerRot = 3336;
  RPMPlanMot = 0;
  lastEncPos = 0;
  currentPlanMotPos = 0;
  lastPlanMotPos = 0;
  deltaPlanMotPos = 0;
  derPlanMotPos = 0;
  motorSpeed = 0;
  intmotorSpeed = 0;
  lastPerror = 0;
  Perror = 0;
  Ierror = 0;
  Derror = 0;
  hubPerror = 0;
  hubIerror = 0;
  hubDerror = 0;
  lastHubPerror = 0;
  PWheelGain = 2.5;
  DWheelGain = -.05;  
  cutoff = 20;
}

returnVariables wheelControl::calculate(float newDesiredRPM, float desiredAngle, float avgDeltaTime, unsigned long lastInterrupt, int encPos) {
  using namespace std;

  /*
 * The next section looks to see if the desired RPM is in the same direction or not. IF it is not, it will set the 
 *  desired RPM to 0 this pass and then next pass if will switch the forward and back variable and the desired RPM.
 *  By setting the desried RPM to 0, we also we restart the interrupt logic and go open loop on the initial speedcheck
 *  value (set later on). 
 */

    if (newDesiredRPM < 0)
    {
      if (lastDesiredRPM > 0) {
        // current positive new negative
        desiredRPM = 0;
      }
      else {
        // current negative new negative
        forwardBackward = LOW;
        desiredRPM =newDesiredRPM*-1;
      }
    }
    else {
      if (lastDesiredRPM < 0) {
        // current negative new positive
        desiredRPM = 0; 
      }
      else {
        //current positive new positive
        desiredRPM = newDesiredRPM;
        forwardBackward = HIGH;
      }
    }
    desiredHubRPM = desiredRPM;
    lastDesiredRPM = newDesiredRPM;
    

  /* 
 *  This if statement is looking for the case that the wheel has stopped and we are not getting any more interrupts, 
 *  and averagedeltatime is not valid (old). So, in this case we will set the current RPM to zero. If not, then we will
 *  calcultate the RPM - the 60,000 is to convert from millisec to minutes.
 */
    if (( millis() - lastInterrupt) > 100) {
      curRPM = 0;
    }
    else {
      curRPM = (1 / ((float)avgDeltaTime * pulsesPerRot)) * 60000;
    }
// IF the wheel has stopped, we need to do a new start up sequence.

    if (curRPM <= cutoff) {
      backToZero = true;
    }else{
      backToZero = false;
    }

    //Serial.print("currentRPM ");
    //Serial.println(curRPM);
// Calculate the delta (error) in RPM and in the derivative of RPM.

    RPMerror = desiredRPM - curRPM;
    DerRPMerror = curRPM - lastcycleRPM;

//          speedCheck = desiredRPM;
//          if(desiredRPM == 0){
//            speedCheck = 0;
//          }

        hubKp = 0.25;
        hubKd = 0.55;
        //hubKi = 1.0;
        float boostGainPos = 250;
        float boostGainNeg = 0;
        float limHubPerror = 0;
        float perrorLimit = 10;
        hubPerror = (desiredRPM - curRPM);
        hubDerror = (hubPerror - lastHubPerror);
//        if(curRPM == 0){
//          hubDerror = 0;
//        }
        lastHubPerror = hubPerror;
        if(hubPerror < 0){//Takes longer to stop so it naturally undershoots, so we slowdown slower. slow PID slower
          hubPerror = hubPerror * .3;
        }
        if(hubPerror > perrorLimit){
          limHubPerror = perrorLimit;
        }else if(hubPerror < -perrorLimit){
          limHubPerror = -perrorLimit*.7;//slow limit slower
        }else{
          limHubPerror = hubPerror;
        }
        speedCheckFloat = limHubPerror * hubKp + hubDerror * hubKd + lastSpeedCheckFloat;
        if(speedCheckFloat > 250){
          speedCheckFloat = 250;
        }
        if(speedCheckFloat < 0){
          speedCheckFloat = 0;
        }

        
        lastSpeedCheckFloat = speedCheckFloat;
        if (desiredRPM/curRPM > 1.3){
          speedCheck = boostGainPos;
        }else if (curRPM/desiredRPM > 1.3){
          speedCheck = boostGainNeg;
        }else{
          speedCheck = long(speedCheckFloat + 0.5);
        }

          //for logging
        hubIerror *=hubKi;
        hubPerror *=limHubPerror * hubKp;
        hubDerror *=hubKd;
        if(desiredRPM == 0){
          speedCheck = 0;
        } 

  

//    //HUB PD START//
//    hubKp = .01;
//    hubKd = .1;
//    hubPerror = desiredRPM - curRPM;
//    hubDerror = hubPerror - lasthubPerror;
//    lastCycleSpeedCheck = speedCheck;
//    speedCheckfloat = (hubKp * hubPerror) + (hubKd * hubDerror);
//    speedCheck = (int)speedCheckfloat + (int)lastCycleSpeedCheck;
//    if(desiredRPM == 0){
//      speedCheck = 0;
//    }
//    lasthubPerror = hubPerror;
    //HUB PD END//

//        hubKp = 5;
//        hubKd = 2;
//        hubKi = .8;


////PID_stepUp
//        hubKp = 4;
//        hubKd = 1.5;
//        hubKi = .85;
//        int slowHubKp = -.25;
//        int idleHubKi = .5;
//        int boostGain = 250;
//        hubPerror = (desiredRPM - curRPM);
//        if(hubPerror < 0 ){hubPerror = hubPerror * slowHubKp;}//if slowing down cut Perror in half
//        hubDerror = (hubPerror - lastHubPerror);
//        lastHubPerror = hubPerror;
//        if(abs(curRPM/desiredRPM) > .8){
//          if(hubIerror == 0){hubIerror = (curRPM-desiredRPM)*idleHubKi;}
//          if(abs(desiredRPM/curRPM) < .7){
//            speedCheck = boostGain *-1;
//          }else{
//            hubIerror +=hubPerror;
//            speedCheck = hubPerror*hubKp + hubDerror*hubKd + hubIerror*hubKi+curRPM;
//          }
//        }else{
//          hubIerror = 0;
//          speedCheck = boostGain;
//          if(hubIerror == 0){hubIerror = (curRPM-desiredRPM)*idleHubKi;}
//        }
//        //for logging
//        hubIerror *=hubKi;
//        hubPerror *=hubKp;
//        hubDerror *=hubKd;
//        if(desiredRPM == 0){
//          speedCheck = 0;
//        } 
      
      

//      //SPEEDING UP
//      if(curRPM < desiredRPM){
//        int boostGain = 10;
//        int boost = 0;
//        hubKi = .6;
//        hubPerror = desiredRPM - curRPM;
//        if(abs(curRPM/desiredRPM < .7)){// far away go full power or 0
//          boost = hubPerror*boostGain;
//          hubIerror = 0;
//        }else{//we only want i error to give extra juice if we are low on batt
//          hubIerror += hubPerror;
//          hubIerror *= hubKi;
//        }
//        if(desiredRPM != 0){
//          speedCheck = desiredRPM + 32 + boost;
//          speedCheck = speedCheck + hubIerror;
//        }else{
//          speedCheck = 0;
//        }
//      }
//      //SLOWING DOWN
//      if(curRPM > desiredRPM){
//        int boostGain = 12;
//        int boost = 0;
//        hubKi = -.4;
//        hubPerror = desiredRPM - curRPM;
//        if(abs(desiredRPM/curRPM < .68)){// far away go  0
//          boost = hubPerror*boostGain;
//          hubIerror = 0;
//        }else{//we only want i error to give extra juice if we are low on batt
//          hubIerror += hubPerror;
//          hubIerror *= hubKi;
//        }
//        if(desiredRPM != 0){
//          speedCheck = desiredRPM + 32 + boost;
//          speedCheck = speedCheck + hubIerror;
//        }else{
//          speedCheck = 0;
//        }
//      }



/*
 * THis is the logic for the planetary motor - setting the angle of the wheel
 */

    deltaEncPos = (float)(encPos - lastEncPos); // change in encoder position is used for RPM

    currentPlanMotPos = lastPlanMotPos + (deltaEncPos * 360.0 / cyclesPerRot);  //calculates current position of wheel

//    // We need to keep the angle between +- 180 deg
//    
//    if (currentPlanMotPos > 180.0) {
//      currentPlanMotPos = currentPlanMotPos - 360.0;
//    }
//    
//    if (currentPlanMotPos < -180.0) {
//      currentPlanMotPos = currentPlanMotPos + 360.0;
//    }

    lastPlanMotPos = currentPlanMotPos;  // used to calculate delta encoder pos, which is use to calcualte RPM
    
    RPMPlanMot = (deltaEncPos / cyclesPerRot) * (float)Hz * 60.0; //RPM calculation
    
    lastEncPos = encPos;
    
    float desiredWheelPos = desiredAngle; //Assumption that the wheel position is really the desired angle 
    float currentWheelPos = currentPlanMotPos; //Same type of assumption
    float deltaWheelPos = desiredWheelPos - currentWheelPos;  // this is the proportional error for the PD controller
    
////    this might be necessary depending upon overall wheel logic
//     if (deltaWheelPos > 180.0) {
//      deltaWheelPos = deltaWheelPos - 360.0;
//    }
//    
//    if (deltaWheelPos < -180.0) {
//      deltaWheelPos = deltaWheelPos + 360.0;
//    }

    float RPM = RPMPlanMot;
    derPlanMotPos = RPM * 6;  //  360 deg/min / 60sec/min = deg/sec  I hope. this is the derivative error for the PD controller


    //PLANETARY PID START//
    float Perror, Ierror, Derror;
    float Kp=5, Ki=6, Kd=.1;
    Perror = desiredWheelPos-currentWheelPos;//calculate proportional term
    Derror = Perror - lastPerror;//calculate derivative term
    Ierror += Perror;//calculate integral term
    motorSpeed = (Kp * Perror) + (Ki * Ierror) + (Kd * Derror);//total gain (pwm written to motor)
    //if((Perror < 1) && (Perror > -1)){motorSpeed = 0;}//do not increment the integral portion if +-1 degree of accuracy achieved
    motorSpeed = constrain(motorSpeed, -100, 100);//constrain floor/ceiling values to fit within +-255(8bit)
    intmotorSpeed = (long)(abs(motorSpeed));//convert to long
    lastPerror = Perror;//set last proportional term for next derivative calculation.
    //PLANETARY PID END
    if (motorSpeed >= 0) {//set appropriate direction for motors to turn since we cant send negatives.
      PlanMotDir = HIGH;
    }
    else {
      PlanMotDir = LOW;
    }
    

// return the desired parameters  
  struct returnVariables returnStruct;
  returnStruct.speedCheck = speedCheck; 
  returnStruct.forwardBackward = forwardBackward;
  returnStruct.currentWheelRPM = curRPM;
  returnStruct.currentWheelAngle = currentPlanMotPos;//Changed to current
  returnStruct.motorSpeed = intmotorSpeed;
  returnStruct.planMotorDirection = PlanMotDir;
  returnStruct.backToZero = backToZero;
  returnStruct.hubIerror = speedCheckFloat;
  returnStruct.hubPerror = int(hubPerror);
  //returnStruct.hubPerror = limHubPerror;
  returnStruct.hubDerror = int(hubDerror);
  returnStruct.desiredHubRPM = int(desiredHubRPM);
  return returnStruct;   
}

