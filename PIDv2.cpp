//
//#include "PIDv2.h"
//#include "IMU6050_.h"
//#include "Receiver.h"
//PIDv2::PIDv2(IMU6050_ *imuRef, Receiver *rxRef) {
//    this->imu = imuRef;
//    this->rx = rxRef;
//  }
//
//int8_t PIDv2::sign(float x) {
//  return (x > 0) - (x < 0); //https://stackoverflow.com/questions/14579920/fast-sign-of-integer-in-c
//}
//
//void PIDv2::getPID(float dps_roll, float dps_pitch, float dps_yaw, float *roll, float *pitch, float *yaw) {
//  *roll = calcPID(dps_roll, imu->getRollV(), &iMem_roll, PID_MAX_ROLL, &prevErr_roll, P_ROLL, I_ROLL, D_ROLL);
//  *pitch = calcPID(dps_pitch, imu->getPitchV(), &iMem_pitch, PID_MAX_PITCH, &prevErr_pitch, P_PITCH, I_PITCH, D_PITCH);
//  *yaw = calcPID(dps_yaw, imu->getYawV(), &iMem_yaw, PID_MAX_YAW, &prevErr_yaw, P_YAW, I_YAW, D_YAW);
//}
//
//
//float PIDv2::calcPID(float dps_setpt, float dps_actual, float *Ival, int maxPID, float *prevErr, float P, float I, float D) {
//  float currErr = (-1.0) * (dps_actual - dps_setpt);
//  *Ival += I * currErr;
//  //if (*Ival > maxPID) *Ival = maxPID;
//  //else if (*Ival < (-1) * maxPID) *Ival = (-1) * maxPID;
//
//  float pid = P * currErr + *Ival + D * (currErr - *prevErr);
//  //if (pid > maxPID) pid = maxPID;
//  //else if (pid < (-1) * maxPID) pid = (-1) * maxPID;
//
//  *prevErr = currErr;
//  return pid;
//}
//
//void ::PIDv2::resetI() {
//  iMem_roll = 0;
//  iMem_pitch = 0;
//  iMem_yaw = 0;
//}
//
//void PIDv2::getMotorValuesDumb(float setpoint_roll, float setpoint_pitch, float setpoint_yaw, float thr, unsigned int *m0, unsigned int *m1, unsigned int *m2, unsigned int *m3, boolean toString) {
//  float rollPID, pitchPID, yawPID;
//  getPID(setpoint_roll, setpoint_pitch, setpoint_yaw, &rollPID, &pitchPID, &yawPID);
////  if (toString) {
////    Serial.print("rPID:" + String(rollPID) + "\t");
////    Serial.print("pPID:" + String(pitchPID) + "\t");
////    Serial.print("yPID:" + String(yawPID) + "\n");
////  }
//
//  int roll = rollPID * ROLL_AMP;
//  int pitch = pitchPID * PITCH_AMP;
//  int yaw = yawPID * YAW_AMP;
//
//    if (toString) {
//    Serial.print("rPID:" + String(roll) + "\t");
//    Serial.print("pPID:" + String(pitch) + "\t");
//    Serial.print("yPID:" + String(yaw) + "\t");
//  }
//  
//  
//  *m1 = thr - pitch - roll - yaw; //Calculate the pulse for esc 1 (front-right - CCW)
//  *m2 = thr + pitch - roll + yaw; //Calculate the pulse for esc 2 (rear-right - CW)
//  *m3 = thr + pitch + roll - yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
//  *m0 = thr - pitch + roll + yaw; //Calculate the pulse for esc 4 (front-left - CW)
//
//  if (*m0 < THR_IDLE) *m0 = THR_IDLE;
//  if (*m0 > 1000) *m0 = 1000;
//  if (*m1 < THR_IDLE) *m1 = THR_IDLE;
//  if (*m1 > 1000) *m1 = 1000;
//  if (*m2 < THR_IDLE) *m2 = THR_IDLE;
//  if (*m2 > 1000) *m2 = 1000;
//  if (*m3 < THR_IDLE) *m3 = THR_IDLE;
//  if (*m3 > 1000) *m3 = 1000;
//}
//
//void PIDv2::getMotorValues(float setpoint_roll, float setpoint_pitch, float setpoint_yaw, float thr, uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, boolean toString) {
//  float roll, pitch, yaw;
//  getPID(setpoint_roll, setpoint_pitch, setpoint_yaw, &roll, &pitch, &yaw);
//  if (toString) {
//    Serial.print("rPID:" + String(roll) + "\t");
//    Serial.print("pPID:" + String(pitch) + "\t");
//    Serial.print("yPID:" + String(yaw) + "\n");
//  }
//  
//  //Serial.println(abs((roll/ROLL_PID_MAX)));
//  uint16_t rollDelta = abs((roll/PID_MAX_ROLL) * 65535);
//  boolean rollPositive = sign(roll) == -1 ? 0 :  1;
//  //Serial.println(rollDelta);
//  
//  uint16_t pitchDelta = abs((pitch/PID_MAX_PITCH) * 65535);
//  boolean pitchPositive = sign(pitch) == -1 ? 1 :  0;
//
//  uint16_t yawDelta = abs((yaw/PID_MAX_PITCH) * 65535);
//  boolean yawPositive = sign(yaw) == -1 ? 0 :  1;
//  
//  uint16_t highRoll, lowRoll, highPitch, lowPitch, highYaw, lowYaw;
//  
//  getDeltas(&highRoll, &lowRoll, rollDelta/2, thr);
//  getDeltas(&highPitch, &lowPitch, pitchDelta/2, thr);
//  getDeltas(&highYaw, &lowYaw, yawDelta/2, thr);
//  //Serial.println(highRoll);
//
//  uint32_t m0temp = 0; uint32_t m1temp = 0; uint32_t m2temp = 0; uint32_t m3temp = 0;
////      Serial.print("H:");
////      Serial.print(highRoll);
////      Serial.print("\tL:");
////      Serial.println(lowRoll);  
//  if (rollPositive) {
//    m0temp += highRoll;
//    m3temp += highRoll;
//    m1temp += lowRoll;
//    m2temp += lowRoll;
//  } else {
//    m0temp += lowRoll;
//    m3temp += lowRoll;
//    m1temp += highRoll;
//    m2temp += highRoll;
//  }
//  
//   
//
//  if (pitchPositive) {
//    m0temp += highPitch;
//    m1temp += highPitch;
//    m2temp += lowPitch;
//    m3temp += lowPitch;
//  } else {
//    m0temp += lowPitch;
//    m1temp += lowPitch;
//    m2temp += highPitch;
//    m3temp += highPitch;        
//  }
//
//  if (yawPositive) {
//    m0temp += highYaw;
//    m2temp += highYaw;
//    m1temp += lowYaw;
//    m3temp += lowYaw;
//  } else {
//    m0temp += lowYaw;
//    m2temp += lowYaw;
//    m1temp += highYaw;
//    m3temp += highYaw;
//  }
//
//  *m0 = m0temp/3;
//  *m1 = m1temp/3;
//  *m2 = m2temp/3;
//  *m3 = m3temp/3;
//}
//
//void PIDv2::getDeltas(uint16_t *high, uint16_t *low, uint16_t deltaHalf, uint16_t setpoint_thr) {
//  //Serial.println(deltaHalf + " , " + )
//  if (setpoint_thr > (65535 - deltaHalf)) {
//    *high = 65535;
//    *low = 65535 - 2*deltaHalf + THR_IDLE;
//  } else if (setpoint_thr - THR_IDLE < deltaHalf) {
//    *low = THR_IDLE;
//    *high = 2*deltaHalf + THR_IDLE;
//  } else {
//    *high = setpoint_thr + deltaHalf;
//    *low = setpoint_thr - deltaHalf + THR_IDLE;
//  }
//}
