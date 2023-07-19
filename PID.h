//#include <stdint.h>
//#include "Arduino.h"
//
//class IMU6050_;
//
//    /*positives:
//     * roll: ->
//     * pitch: ^ (pitch up)
//     * yaw: ccw
//     * 
//     * Top Down:
//     * 
//     *    FRONT
//     * m0(ccw) m1(cw)
//     * 
//     * 
//     * m3(cw)  m2(ccw)
//     *    BACK
//     */
//
//class PID {
//  private:
//    const double ROLL_PID_MAX = 10000.0; 
//    const double PITCH_PID_MAX = 10000.0;
//    const double YAW_PID_MAX = 10000.0;
//
//    const float KP_ROLL = 8.0;
//    const float KI_ROLL = 0.010; //.001
//    const float KD_ROLL = 4.50;
//    
//    const float KP_PITCH = 15.0;
//    const float KI_PITCH = 0.010;
//    const float KD_PITCH = 7.50;
//    
//    const float KP_YAW = 3.0;
//    const float KI_YAW = 1.00000;
//    const float KD_YAW = 5.00;
//
//    //const float I_MAX_ERR_THRESH = 10000.0; //when err is above I_MAX_ERR_THRESH deg/s then I will do nothing and maintain its curr val
//    const float I_MIN_ERR_THRESH = 0.0;
//    
//    float setpoint_roll = 0; //deg/s
//    float setpoint_pitch = 0; //deg/s
//    float setpoint_yaw = 0; //deg/s
//    uint16_t setpoint_thr = 32767;
//    uint16_t THR_IDLE = 6001; //THR_IDLE/65535 = percentage 
//
//    float I_pitch = 0;
//    float I_roll = 0;
//    float I_yaw = 0;
//    
//    float prevError_roll = 0;
//    float prevError_pitch = 0;
//    float prevError_yaw = 0;
//    int dataDate = micros(); //micros since data collection
//    IMU6050_ *imu;
//
//    float accumulate(float err, float *I, float deltaS, float K, float *prevErr);
//    float accumulateV2(float err, float *I, float deltaS, float K, float *prevErr);
//    int8_t sign(float x); 
//    void getPID(float *roll, float *pitch, float *yaw);
//    void getMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, float roll, float pitch, float yaw);
//    void getDeltas(uint16_t *high, uint16_t *low, uint16_t deltaHalf);
// 
//  public:
//    PID(IMU6050_ *imuRef);
//    void updateSetpoints(float r, float p, float y, uint16_t thr);
//    void updateMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, bool printOut); 
//    void resetI();
//};
