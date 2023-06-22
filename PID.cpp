#include "PID.h"
#include "IMU6050_.h"
PID::PID(IMU6050_ *imuRef) {
  this->imu = imuRef;
}
//integrator with constant scalar
double PID::accumulate(double err, double *I, double deltaS, float K, double *prevErr) {
  // if (sign(err) != sign(*prevErr)) {
    //*I = 0;
  // }else if(abs(err) < I_MAX_ERR_THRESH && abs(err) > I_MIN_ERR_THRESH) {
  if (abs(*I) < ROLL_PID_MAX) {
    *I = *I + deltaS*err*K;
  }
  if ((sign(*prevErr)) != sign(err)) {
    *I = 0;
  }
    
  // }
  *prevErr = err;
  return *I;
}

int8_t PID::sign(double x) {
  return (x > 0) - (x < 0); //https://stackoverflow.com/questions/14579920/fast-sign-of-integer-in-c
}
//get raw pid value per axis
void PID::getPID(double *roll, double *pitch, double *yaw) {
  
  double deltaS = (micros() - dataDate)/1000000.0;
  double rollErr = setpoint_roll - imu->getRollV();
  double pitchErr = setpoint_pitch - imu->getPitchV();
  double yawErr = setpoint_yaw - imu->getYawV();
  yawErr = abs(yawErr) > 3 ? yawErr : 0;
  //Serial.print("YERR:" + String(yawErr) + "\t");
  double rollPID = (rollErr) * KP_ROLL + accumulate(rollErr, &I_roll, deltaS, KI_ROLL, &prevError_roll) + (-1.0)*imu->getRollA()*KD_ROLL;
  double pitchPID = (pitchErr) * KP_PITCH + accumulate(pitchErr, &I_pitch, deltaS, KI_PITCH, &prevError_pitch) + (-1.0)*imu->getPitchA()*KD_PITCH;
  double yawPID = (yawErr) * KP_YAW + accumulate(yawErr, &I_yaw, deltaS, KI_YAW, &prevError_yaw) + (-1.0)*imu->getYawA()*KD_YAW; 
  
  *roll = abs(rollPID) > ROLL_PID_MAX ? ROLL_PID_MAX*sign(rollPID) : rollPID;
  *pitch = abs(pitchPID) > PITCH_PID_MAX ? PITCH_PID_MAX*sign(pitchPID) : pitchPID;
  *yaw = abs(yawPID) > YAW_PID_MAX ? YAW_PID_MAX*sign(yawPID) : yawPID;
}

void PID::getMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, double roll, double pitch, double yaw) {
  //Serial.println(abs((roll/ROLL_PID_MAX)));
  uint16_t rollDelta = abs((roll/ROLL_PID_MAX) * 65535);
  boolean rollPositive = sign(roll) == -1 ? 0 :  1;
  //Serial.println(rollDelta);
  
  uint16_t pitchDelta = abs((pitch/PITCH_PID_MAX) * 65535);
  boolean pitchPositive = sign(pitch) == -1 ? 1 :  0;

  uint16_t yawDelta = abs((yaw/YAW_PID_MAX) * 65535);
  boolean yawPositive = sign(yaw) == -1 ? 0 :  1;
  
  uint16_t highRoll, lowRoll, highPitch, lowPitch, highYaw, lowYaw;
  
  getDeltas(&highRoll, &lowRoll, rollDelta/2);
  getDeltas(&highPitch, &lowPitch, pitchDelta/2);
  getDeltas(&highYaw, &lowYaw, yawDelta/2);
  //Serial.println(highRoll);

  uint32_t m0temp = 0; uint32_t m1temp = 0; uint32_t m2temp = 0; uint32_t m3temp = 0;
//      Serial.print("H:");
//      Serial.print(highRoll);
//      Serial.print("\tL:");
//      Serial.println(lowRoll);  
  if (rollPositive) {
    m0temp += highRoll;
    m3temp += highRoll;
    m1temp += lowRoll;
    m2temp += lowRoll;
  } else {
    m0temp += lowRoll;
    m3temp += lowRoll;
    m1temp += highRoll;
    m2temp += highRoll;
  }
  
   

  if (pitchPositive) {
    m0temp += highPitch;
    m1temp += highPitch;
    m2temp += lowPitch;
    m3temp += lowPitch;
  } else {
    m0temp += lowPitch;
    m1temp += lowPitch;
    m2temp += highPitch;
    m3temp += highPitch;        
  }

  if (yawPositive) {
    m0temp += highYaw;
    m2temp += highYaw;
    m1temp += lowYaw;
    m3temp += lowYaw;
  } else {
    m0temp += lowYaw;
    m2temp += lowYaw;
    m1temp += highYaw;
    m3temp += highYaw;
  }

  *m0 = m0temp/3;
  *m1 = m1temp/3;
  *m2 = m2temp/3;
  *m3 = m3temp/3;
}

void PID::getDeltas(uint16_t *high, uint16_t *low, uint16_t deltaHalf) {
  //Serial.println(deltaHalf + " , " + )
  if (setpoint_thr > (65535 - deltaHalf)) {
    *high = 65535;
    *low = 65535 - 2*deltaHalf + THR_IDLE;
  } else if (setpoint_thr - THR_IDLE < deltaHalf) {
    *low = THR_IDLE;
    *high = 2*deltaHalf + THR_IDLE;
  } else {
    *high = setpoint_thr + deltaHalf;
    *low = setpoint_thr - deltaHalf + THR_IDLE;
  }
}

void PID::updateSetpoints(double r, double p, double y, uint16_t thr) {
  setpoint_roll = r;
  setpoint_pitch = p;
  setpoint_yaw = y;
  if (thr < THR_IDLE) {
    setpoint_thr = THR_IDLE;
  } else {
    setpoint_thr = thr;
  } 
  
}

void PID::updateMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, bool printOut) {
  //recvr->getSetpoints(&setpoint_roll, &setpoint_pitch, &setpoint_yaw, &setpoint_thr);
  double r, p, y;
  getPID(&r, &p, &y);
  if (printOut) {
    Serial.print("r:" + String(r) + "\t" + "p:" + String(p) + "\t" + "y:" + String(y) + "\n");
  }
  getMotorValues(m0, m1, m2, m3, r, p, y);
} 

 void PID::resetI() {
    I_pitch = 0;
    I_roll = 0;
    I_yaw = 0;
    prevError_roll = 0;
    prevError_pitch = 0;
    prevError_yaw = 0;
 }
