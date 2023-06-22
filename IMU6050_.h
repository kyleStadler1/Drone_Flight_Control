#include "windowFilter.h"
#include "frequencyFilter.h"
#include "MPU6050.h"
#include <Wire.h>
class IMU6050_ {
//My MPU6050 tests:
//1khz refresh rate; 360 us read time; 
/*
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * 
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
  */
  private:
    const uint8_t IMU_ADDR = 0x68;
    const unsigned int DLFP_CFG = 3;
    const unsigned int DPS = 1000;
    const float DPS_LSB = 32.8;
    const uint8_t FS_SEL = 2;
    MPU6050 imu;
    int dataDate = micros(); //micros since data collection
    float rollVelo = 0.0; //deg/sec
    float rollAccel = 0.0; //deg/sec2
    float pitchVelo = 0.0; //deg/sec
    float pitchAccel = 0.0; //deg/sec2
    float yawVelo = 0.0; //deg/sec
    float yawAccel = 0.0; //deg/sec2
    float rollVeloRaw = 0.0; //deg/sec
    float rollAccelRaw = 0.0; //deg/sec2
    float pitchVeloRaw = 0.0; //deg/sec
    float pitchAccelRaw = 0.0; //deg/sec2
    float yawVeloRaw = 0.0; //deg/sec
    float yawAccelRaw = 0.0; //deg/sec2
    uint8_t veloFilterSize;
    uint8_t accelFilterSize;
    windowFilter *veloFilter;
    windowFilter *accelFilter;
    //windowFilter *accelFilterAux;
    frequencyFilter *accelFreqFilter;

  public:
    IMU6050_(uint8_t veloFilterSize, uint8_t accelFilterSize);   
    void updateRotationData();//updates 3 axis rotational data of angular velovity(deg/s) and angular acceleration(deg/s2)
    void resetData();
    float getRollV();
    float getPitchV();
    float getYawV();
    float getRollA();
    float getPitchA();
    float getYawA();
    float getRollVRaw();
    float getPitchVRaw();
    float getYawVRaw();
    float getRollARaw();
    float getPitchARaw();
    float getYawARaw();
    float getDataDate(); 
    //toStrings
    String angularVelocityToString();
    String angularAccelerationToString();  //toString of all angular accelerations with truncated degree/s2 values

   
};
