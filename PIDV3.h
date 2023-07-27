#include "IMU6050_.h"
#include "Receiver.h"
//class IMU6050_;
//class Receiver;

class PIDV3 {
  private: 

// working: .5,.5 imu and then ps at .4, no I, .6d, 
    float pid_p_gain_roll = .4; //1.3              //Gain setting for the roll P-controller
    float pid_i_gain_roll = 0.001; //.04              //Gain setting for the roll I-controller
    float pid_d_gain_roll = 1.5; //5.0             //Gain setting for the roll D-controller
    int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)
    
    float pid_p_gain_pitch = .4;  //Gain setting for the pitch P-controller.
    float pid_i_gain_pitch = 0.001; //.04 //Gain setting for the pitch I-controller.
    float pid_d_gain_pitch = 1.5;  //3Gain setting for the pitch D-controller.
    int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)
    
    float pid_p_gain_yaw = 2;  //2              //Gain setting for the pitch P-controller. //4.0
    float pid_i_gain_yaw = 0.001; //.02              //Gain setting for the pitch I-controller. //0.02
    float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
    int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
    unsigned int setpoint_thr;
  
    IMU6050_ *imu;
    Receiver *rx;
    float pid_error_temp;
    float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
    float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
    float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;


  public:
    PIDV3(IMU6050_ *imuRef, Receiver *rxRef); 
    void getMotorValues(unsigned int *m0, unsigned int *m1, unsigned int *m2, unsigned int *m3, boolean toString);
    void calculate_pid();
    void PIDV3::updateData();
    void PIDV3::resetI();
  
};
