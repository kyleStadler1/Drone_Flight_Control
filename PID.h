#include <stdint.h>
#include "Arduino.h"

class IMU6050_;

    /*positives:
     * roll: ->
     * pitch: ^ (pitch up)
     * yaw: ccw
     * 
     * Top Down:
     * 
     *    FRONT
     * m0(ccw) m1(cw)
     * 
     * 
     * m3(cw)  m2(ccw)
     *    BACK
     */

class PID {
  private:
    const double ROLL_PID_MAX = 10000.0; 
    const double PITCH_PID_MAX = 10000.0;
    const double YAW_PID_MAX = 10000.0;

    const float KP_ROLL = 40.0;
    const float KI_ROLL = 0.000; //.001
    const float KD_ROLL = 10.50;
    
    const float KP_PITCH = 20.0;
    const float KI_PITCH = 0.000;
    const float KD_PITCH = 10.50;
    
    const float KP_YAW = 20.0;
    const float KI_YAW = 0.00000;
    const float KD_YAW = 10.00;

    //const float I_MAX_ERR_THRESH = 10000.0; //when err is above I_MAX_ERR_THRESH deg/s then I will do nothing and maintain its curr val
    const float I_MIN_ERR_THRESH = 0.0;
    
    double setpoint_roll = 0; //deg/s
    double setpoint_pitch = 0; //deg/s
    double setpoint_yaw = 0; //deg/s
    uint16_t setpoint_thr = 32767;
    uint16_t THR_IDLE = 5001; //THR_IDLE/65535 = percentage 

    double I_pitch = 0;
    double I_roll = 0;
    double I_yaw = 0;
    
    double prevError_roll = 0;
    double prevError_pitch = 0;
    double prevError_yaw = 0;
    int dataDate = micros(); //micros since data collection
    IMU6050_ *imu;

    double accumulate(double err, double *I, double deltaS, float K, double *prevErr);
    int8_t sign(double x); 
    void getPID(double *roll, double *pitch, double *yaw);
    void getMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, double roll, double pitch, double yaw);
    void getDeltas(uint16_t *high, uint16_t *low, uint16_t deltaHalf);
 
  public:
    PID(IMU6050_ *imuRef);
    void updateSetpoints(double r, double p, double y, uint16_t thr);
    void updateMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, bool printOut); 
    void resetI();
};
