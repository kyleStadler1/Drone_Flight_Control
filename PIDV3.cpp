#include "PIDv3.h"

PIDV3::PIDV3(IMU6050_ *imuRef, Receiver *rxRef) {
    this->imu = imuRef;
    this->rx = rxRef;
  }

void PIDV3::calculate_pid(){
  
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  //

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

//    if ((pid_last_pitch_d_error > 0 && pid_error_temp < 0) || (pid_last_pitch_d_error < 0 && pid_error_temp > 0)) {
//    pid_i_mem_pitch = 0;
//  }

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void PIDV3::updateData() {
  gyro_roll_input = imu->getRollV();
  gyro_pitch_input = imu->getPitchV();
  gyro_yaw_input = imu->getYawV();
  rx->getSetpoints(&pid_roll_setpoint, &pid_pitch_setpoint, &pid_yaw_setpoint, &setpoint_thr);
//  Serial.print("r: ");
//  Serial.print(pid_roll_setpoint);
//  Serial.print("\t");
//  Serial.print("p: ");
//    Serial.print(pid_pitch_setpoint);
//  Serial.print("\t");
//  Serial.print("y: ");
//    Serial.print(pid_yaw_setpoint);
//  Serial.print("\t");
//  Serial.print("t: ");
//    Serial.print(setpoint_thr);
//  Serial.print("\n");
}

void PIDV3::getMotorValues(unsigned int *m0, unsigned int *m1, unsigned int *m2, unsigned int *m3, boolean toString) {
  updateData();
  if (rx->getArm() && !rx->getFailsafe()) {
    calculate_pid();
    int esc_1;
    int esc_2;
    int esc_3;
    int esc_4;
    if (setpoint_thr > 1800) setpoint_thr = 1800;                                   //We need some room to keep full control at full throttle.
      esc_1 = setpoint_thr + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
      esc_2 = setpoint_thr - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
      esc_3 = setpoint_thr - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc_4 = setpoint_thr + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
  
      if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
      if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
      if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
      if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.
  
      if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
      if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
      if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
      if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
    
  
  
    *m0 = esc_4;
    *m1 = esc_1;
    *m2 = esc_2;
    *m3 = esc_3;
  } else {
    *m0 = 1000;
    *m1 = 1000;
    *m2 = 1000;
    *m3 = 1000;
    resetI();
  }
  

}

void PIDV3::resetI() {
  pid_i_mem_roll = 0;
  pid_i_mem_pitch = 0;
  pid_i_mem_yaw = 0;
}
