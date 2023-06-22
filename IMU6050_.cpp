#include "IMU6050_.h"

IMU6050_::IMU6050_(uint8_t veloFilterSize, uint8_t accelFilterSize) {
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);  // Start communication with MPU6050
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set MPU6050 to sleep mode (reset value)
  Wire.endTransmission(true);
  delay(100);
  //
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Set MPU6050 to active mode
  Wire.endTransmission(true);
  delay(100);
  // Enable Data Ready interrupt
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x38);  // INT_ENABLE register address
  Wire.write(0x01);  // Enable Data Ready interrupt
  Wire.endTransmission(true);
  delay(100);
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.print("IMU init err\n");
    while (1){;};
  } else {
    imu.setDLPFMode(DLFP_CFG);
    imu.setFullScaleGyroRange(FS_SEL);
    //Serial.print("IMU initted!\n");
  }  
  //this->filterSize = filterSize;
//  rVels = new double[filterSize];
//  pVels = new double[filterSize];
//  yVels = new double[filterSize];
//  for (int i = 0; i < filterSize; i++) {
//    rVels[i] = 0.0;
//    pVels[i] = 0.0;
//    yVels[i] = 0.0;
//  }
  veloFilter = new windowFilter(veloFilterSize);
  accelFilter = new windowFilter(accelFilterSize);
  //accelFilterAux = new windowFilter(accelFilterSize);
  //accelFreqFilter = new frequencyFilter(3);

  
}

void IMU6050_::updateRotationData() {
  int16_t x, y, z;
  imu.getRotation(&x, &y, &z);
  //Serial.println(*DPS_LSB);
  float deltaS = (micros() - dataDate)/1000000.0;
  float rollVeloRawNew = (float)x / (float)DPS_LSB + 0.368903;
  float pitchVeloRawNew = (float)y / (float)DPS_LSB - 0.398171;
  float yawVeloRawNew = (float)z / (float)DPS_LSB + 0.725000;

  float rollVeloFilt;
  float pitchVeloFilt;
  float yawVeloFilt;
  veloFilter->filter(rollVeloRawNew, pitchVeloRawNew, yawVeloRawNew, &rollVeloFilt, &pitchVeloFilt, &yawVeloFilt);

  rollAccelRaw = (rollVeloFilt - rollVelo)/deltaS;
  pitchAccelRaw = (pitchVeloFilt - pitchVelo)/deltaS;
  yawAccelRaw = (yawVeloFilt - yawVelo)/deltaS;
  
  rollVelo = rollVeloFilt;
  pitchVelo = pitchVeloFilt;
  yawVelo = yawVeloFilt;
  rollVeloRaw = rollVeloRawNew;
  pitchVeloRaw = pitchVeloRawNew;
  yawVeloRaw = yawVeloRawNew;

  
  accelFilter->filter(rollAccelRaw, pitchAccelRaw, yawAccelRaw, &rollAccel, &pitchAccel, &yawAccel);
  //accelFilterAux->filter(rollAccelRaw, pitchAccelRaw, yawAccelRaw, &rollAccel, &pitchAccel, &yawAccel);
  //accelFreqFilter->filter(rollAccel, pitchAccel, yawAccel, &rollAccel, &pitchAccel, &yawAccel);
//  rollAccel = abs(rollAccel) > 100 ? rollAccel : 0;
//  pitchAccel = abs(pitchAccel) > 100 ? pitchAccel : 0;
//  yawAccel = abs(yawAccel) > 100 ? yawAccel : 0;
//  
  

  dataDate = micros();
}

void IMU6050_::resetData() {
  rollVelo = 0.0; //deg/sec
  rollAccel = 0.0; //deg/sec2
  pitchVelo = 0.0; //deg/sec
  pitchAccel = 0.0; //deg/sec2
  yawVelo = 0.0; //deg/sec
  yawAccel = 0.0; //deg/sec2
  rollVeloRaw = 0.0; //deg/sec
  rollAccelRaw = 0.0; //deg/sec2
  pitchVeloRaw = 0.0; //deg/sec
  pitchAccelRaw = 0.0; //deg/sec2
  yawVeloRaw = 0.0; //deg/sec
  yawAccelRaw = 0.0; //deg/sec2
  veloFilter->resetFilter();
  accelFilter->resetFilter();
}

String IMU6050_::angularVelocityToString() { //toString of all angular velocities with truncated degree/s values
  char buffer[80];
  sprintf(buffer, "rollV:%f\tpitchV:%f\tyawV:%f\t", float(rollVelo), float(pitchVelo), float(yawVelo));
  return buffer;
}

String IMU6050_::angularAccelerationToString() { //toString of all angular accelerations with truncated degree/s2 values
  char buffer[80];
  sprintf(buffer, "rollA:%f\tpitchA:%f\tyawA:%f\n", (rollAccel), (pitchAccel), (yawAccel));
  return buffer;
}

float IMU6050_::getRollV() {return rollVelo;};
float IMU6050_::getPitchV() {return pitchVelo;};
float IMU6050_::getYawV() {return yawVelo;};
float IMU6050_::getRollA() {return rollAccel;};
float IMU6050_::getPitchA() {return pitchAccel;};
float IMU6050_::getYawA() {return yawAccel;};
float IMU6050_::getRollVRaw() {return rollVeloRaw;};
float IMU6050_::getPitchVRaw() {return pitchVeloRaw;};
float IMU6050_::getYawVRaw() {return yawVeloRaw;};
float IMU6050_::getRollARaw() {return rollAccelRaw;};
float IMU6050_::getPitchARaw() {return pitchAccelRaw;};
float IMU6050_::getYawARaw() {return yawAccelRaw;};
float IMU6050_::getDataDate() {return dataDate;}; 
