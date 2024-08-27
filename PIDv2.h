//#include <stdint.h>
//#include "Arduino.h"
//
//class IMU6050_;
//class Receiver;
//
//class PIDv2 {
//  private:
//  IMU6050_ *imu;
//  Receiver *rx;
//  
//  const float P_ROLL = 0.2;
//  const float I_ROLL = 0;
//  const float D_ROLL = .2;//40
//  const int PID_MAX_ROLL = 400;
//  const float P_PITCH = 0.2;
//  const float I_PITCH = 0;
//  const float D_PITCH = .2;//30
//  const int PID_MAX_PITCH = 400;
//  const float P_YAW = 1.5;
//  const float I_YAW = 0.00;
//  const float D_YAW = 0.0;
//  const int PID_MAX_YAW = 400;
//
//  const int ROLL_AMP = 1;//65535 / (2 * 400);
//  const int PITCH_AMP = 1;//65535 / (2 * 400);
//  const int YAW_AMP = 1;//65535 / (2 * 400);
//  
//  uint16_t THR_IDLE = 60; //THR_IDLE/65535 = percentage 
//
//  float prevErr_roll = 0, prevErr_pitch = 0, prevErr_yaw = 0, iMem_roll = 0, iMem_pitch = 0, iMem_yaw = 0;
//
//  public:
//    void getMotorValues(float setpoint_roll, float setpoint_pitch, float setpoint_yaw, float thr, uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, boolean toString);
//    void getMotorValuesDumb(float setpoint_roll, float setpoint_pitch, float setpoint_yaw, float thr, unsigned int *m0, unsigned int *m1, unsigned int *m2, unsigned int *m3, boolean toString);
//    PIDv2(IMU6050_ *imuRef, Receiver *rxRef); 
//    void resetI();
//    
//  private:
//    float calcPID(float dps_setpt, float dps_actual, float *Ival, int maxPID, float *prevErr, float P, float I, float D);
//    void getPID(float dps_roll, float dps_pitch, float dps_yaw, float *roll, float *pitch, float *yaw); 
//    int8_t sign(float x);
//    void getDeltas(uint16_t *high, uint16_t *low, uint16_t deltaHalf, uint16_t thr);
//};
