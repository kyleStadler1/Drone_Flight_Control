#include <Wire.h>
#include "MPU6050.h"
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;
#include "sbus.h"
#include <Servo.h> 
class IMU6050;
class PID_Controller;
class Receiver;

class IMU6050 {
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
    const uint8_t DLFP_CFG = 0;
    const unsigned int DPS = 1000;
    float DPS_LSB;
    uint8_t FS_SEL;
    MPU6050 imu;
    int dataDate = micros(); //micros since data collection
    double rollVelo = 0.0; //deg/sec
    double rollAccel = 0.0; //deg/sec2
    double pitchVelo = 0.0; //deg/sec
    double pitchAccel = 0.0; //deg/sec2
    double yawVelo = 0.0; //deg/sec
    double yawAccel = 0.0; //deg/sec2
    double rollVeloRaw = 0.0; //deg/sec
    double rollAccelRaw = 0.0; //deg/sec2
    double pitchVeloRaw = 0.0; //deg/sec
    double pitchAccelRaw = 0.0; //deg/sec2
    double yawVeloRaw = 0.0; //deg/sec
    double yawAccelRaw = 0.0; //deg/sec2
    uint8_t filterSize;
    double *rVels;
    double *pVels;
    double *yVels;
    
    //double rollVeloExp = 0.0;
    //double rollAccelExp = 0.0;
    
    
    void setRuntimeConsts() {
      float dps_lsb;
      uint8_t fs_sel;
      switch(DPS) {
        case 250:
          dps_lsb = 131.0;
          fs_sel = 0;
          break;
        case 500:
          dps_lsb = 65.5;
          fs_sel = 1;
          break;
        case 1000:
          dps_lsb = 32.8;
          fs_sel = 2;
          break;
        case 2000:
          dps_lsb = 16.4;
          fs_sel = 3;
          break;
        default:
          Serial.print("DPS value is not valid, setting to 1000dps");
          dps_lsb = 32.8;
          fs_sel = 2;
          break;
      }
//      const float DPS_LSB_RTCONST = dps_lsb;
//      const uint8_t FS_SEL_RTCONST = fs_sel;
//      DPS_LSB = &DPS_LSB_RTCONST;
//      FS_SEL = &FS_SEL_RTCONST;
        DPS_LSB = dps_lsb;
        FS_SEL = fs_sel;
    } 
  public:
    IMU6050(int filterSize) {
//      Serial.begin(115200);
//      while(!Serial){};
      setRuntimeConsts();
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
      this->filterSize = filterSize;
      rVels = new double[filterSize];
      pVels = new double[filterSize];
      yVels = new double[filterSize];
      for (int i = 0; i < filterSize; i++) {
        rVels[i] = 0.0;
        pVels[i] = 0.0;
        yVels[i] = 0.0;
      }
    }

    void windowAvgFilter(double rollVeloRaw, double pitchVeloRaw, double yawVeloRaw, double *rollVeloFilt, double *pitchVeloFilt, double *yawVeloFilt) {
      for (int i = filterSize-1; i > 0; i--) {
        rVels[i] = rVels[i-1] + rollVeloRaw;
        pVels[i] = pVels[i-1] + pitchVeloRaw;
        yVels[i] = yVels[i-1] + yawVeloRaw;
      }
      rVels[0] = rollVeloRaw;
      pVels[0] = pitchVeloRaw;
      yVels[0] = yawVeloRaw;
      *rollVeloFilt = rVels[filterSize-1]/(double)filterSize;
      *pitchVeloFilt = pVels[filterSize-1]/(double)filterSize;
      *yawVeloFilt = yVels[filterSize-1]/(double)filterSize;
    }

    //updates 3 axis rotational data of angular velovity(deg/s) and angular acceleration(deg/s2)
    void updateRotationData() {
      int16_t x, y, z;
      imu.getRotation(&x, &y, &z);
      //Serial.println(*DPS_LSB);
      double deltaS = (micros() - dataDate)/1000000.0;
      double rollVeloRawNew = (float)x / DPS_LSB;
      double pitchVeloRawNew = (float)y / DPS_LSB;
      double yawVeloRawNew = (float)z / DPS_LSB;
      rollAccelRaw = (rollVeloRawNew - rollVeloRaw)/deltaS;
      pitchAccelRaw = (pitchVeloRawNew - pitchVeloRaw)/deltaS;
      yawAccelRaw = (yawVeloRawNew - yawVeloRaw)/deltaS;
      rollVeloRaw = rollVeloRawNew;
      pitchVeloRaw = pitchVeloRawNew;
      yawVeloRaw = yawVeloRawNew;
      double rollVeloFilt;
      double pitchVeloFilt;
      double yawVeloFilt;
      windowAvgFilter(rollVeloRawNew, pitchVeloRawNew, yawVeloRawNew, &rollVeloFilt, &pitchVeloFilt, &yawVeloFilt);
      rollAccel = (rollVeloFilt - rollVelo)/deltaS;
      pitchAccel = (pitchVeloFilt - pitchVelo)/deltaS;
      yawAccel = (yawVeloFilt - yawVelo)/deltaS;
      rollVelo = rollVeloFilt;
      pitchVelo = pitchVeloFilt;
      yawVelo = yawVeloFilt;
      dataDate = micros();
    }
    
    void arrToString(double arr[]) {
      Serial.print("\n");
      for (int i = 0; i < 10; i++) {
        Serial.print((double)arr[i]);
        Serial.print("\t");
      }
      Serial.print("\n");
    }
    //getters
    double getRollV() {return rollVelo;};
    double getPitchV() {return pitchVelo;};
    double getYawV() {return yawVelo;};
    double getRollA() {return rollAccel;};
    double getPitchA() {return pitchAccel;};
    double getYawA() {return yawAccel;};
    double getRollVRaw() {return rollVeloRaw;};
    double getPitchVRaw() {return pitchVeloRaw;};
    double getYawVRaw() {return yawVeloRaw;};
    double getRollARaw() {return rollAccelRaw;};
    double getPitchARaw() {return pitchAccelRaw;};
    double getYawARaw() {return yawAccelRaw;};
    double getDataDate() {return dataDate;}; //micros date
    
    //toStrings
    String angularVelocityToString() { //toString of all angular velocities with truncated degree/s values
      char buffer[80];
      sprintf(buffer, "rollV: %d \tpitchV: %d \tyawV: %d \n", int(rollVelo), int(pitchVelo), int(yawVelo));
      return buffer;
    }
    String angularAccelerationToString() { //toString of all angular accelerations with truncated degree/s2 values
      char buffer[80];
      sprintf(buffer, "rollA: %f \tpitchA: %f \tyawA: %f \n", (rollAccel), (pitchAccel), (yawAccel));
      return buffer;
    }
};

class Receiver {
  private:
    bfs:: SbusRx *sbus;
    bfs::SbusData *data_;
    const uint16_t CH_MAX = 1811;
    const uint16_t CH_MIN = 172;
    const uint16_t CH_MID = 992;
    #define FAILSAFE 9
    #define THR 2
    #define YAW 3
    #define PITCH 1
    #define ROLL 0
    #define ARM 4 //aux switch
    #define TURTLE 5
    #define BEEPER 7
    double roll_rate = 0;
    double pitch_rate = 0;
    double yaw_rate = 0;
    double thr_rate = 0;
    uint16_t arm_stick = CH_MID;
    uint16_t turtle_stick = CH_MID;
    uint16_t beeper_stick = CH_MID;
    uint16_t failsafe_stat = 0;
    const double CUBIC_AMP =  0.00000117956886463;
    const double LINEAR_AMP = 0.791208791209;
    
    double rates(uint16_t rawCh) {
      double midPtOffset = ((double)rawCh - (double)CH_MID);
      return ((CUBIC_AMP*pow(midPtOffset, 3) + LINEAR_AMP*(midPtOffset))/2.0);
    }
    
    uint16_t convertThr(uint16_t thr) {
      return (thr - CH_MIN)*65535/CH_MAX;
    }

    boolean update_rates() {
      if(sbus->Read()){
        *data_ = sbus->data();
        //Serial.println(this->data_->ch[ROLL]);
        roll_rate = rates(data_->ch[ROLL]);
        pitch_rate = rates(data_->ch[PITCH]);
        yaw_rate = rates(data_->ch[YAW]);
        thr_rate = convertThr(data_->ch[THR]);
        arm_stick = data_->ch[ARM];
        turtle_stick = data_->ch[TURTLE];
        beeper_stick = data_->ch[BEEPER];
        failsafe_stat = data_->ch[FAILSAFE];
        return true;
      }  
      return false;
      
  }
  public:
    Receiver(boolean invert) {
      sbus = new bfs::SbusRx(&Serial8); //(serialPort, inverted, highspeed)
      data_ = &sbus->data();
      sbus->Begin();
    }
    void getSetpoints(double *r, double *p, double *y, uint16_t *thr) {
      update_rates();
      *r = roll_rate;
      *p = pitch_rate;
      *y = yaw_rate;
      *thr = thr_rate;
    }
    boolean getArm() {
      return arm_stick > CH_MID + 200;
    }
    boolean getFailsafe() {
      return failsafe_stat == 0;
    }


};

class PID_Controller {
  private:
    const double ROLL_PID_MAX = 10000.0; 
    const double PITCH_PID_MAX = 10000.0;
    const double YAW_PID_MAX = 10000.0;

    const float KP_ROLL = 30.0;
    const float KI_ROLL = 0.0;
    const float KD_ROLL = 3.3;
    
    const float KP_PITCH = 30.0;
    const float KI_PITCH = 0.0;
    const float KD_PITCH = 3.3;
    
    const float KP_YAW = 30.0;
    const float KI_YAW = 0.0;
    const float KD_YAW = 3.3;

    const float I_MAX_ERR_THRESH = 50.0; //when err is above I_MAX_ERR_THRESH deg/s then I will do nothing and maintain its curr val
    const float I_MIN_ERR_THRESH = 1.0;
    double setpoint_roll = 0; //deg/s
    double setpoint_pitch = 0; //deg/s
    double setpoint_yaw = 0; //deg/s
    uint16_t setpoint_thr = 32767;
    uint16_t THR_IDLE = 6000; //THR_IDLE/65535 = percentage 

    double I_pitch = 0;
    double I_roll = 0;
    double I_yaw = 0;


    double prevError_roll = 0;
    double prevError_pitch = 0;
    double prevError_yaw = 0;
    int dataDate = micros(); //micros since data collection
    IMU6050 *imu;
    //Receiver *recvr;

    double accumulate(double err, double *I, double deltaS, float K, double *prevErr) {
      if (sign(err) != sign(*prevErr)) {
        *I = 0;
      }else if(abs(err) < I_MAX_ERR_THRESH && abs(err) > I_MIN_ERR_THRESH) {
        *I = *I + deltaS*err*K;
      }
      *prevErr = err;
      return *I;
    }

    int8_t sign(double x) { //https://stackoverflow.com/questions/14579920/fast-sign-of-integer-in-c
      return (x > 0) - (x < 0);
    }

    void getPID(double *roll, double *pitch, double *yaw) {
      double deltaS = (micros() - dataDate)/1000000.0;
      double rollErr = setpoint_roll - imu->getRollV();
      double pitchErr = setpoint_pitch - imu->getPitchV();
      double yawErr = setpoint_yaw - imu->getYawV();
      
      double rollPID = (rollErr) * KP_ROLL + accumulate(rollErr, &I_roll, deltaS, KI_ROLL, &prevError_roll) + (-1.0)*imu->getRollA()*KD_ROLL;
      double pitchPID = (pitchErr) * KP_PITCH + accumulate(pitchErr, &I_pitch, deltaS, KI_PITCH, &prevError_pitch) + (-1.0)*imu->getPitchA()*KD_PITCH;
      double yawPID = (yawErr) * KP_YAW + accumulate(yawErr, &I_yaw, deltaS, KI_YAW, &prevError_yaw) + (-1.0)*imu->getYawA()*KD_YAW; 
      
      *roll = abs(rollPID) > ROLL_PID_MAX ? ROLL_PID_MAX*sign(rollPID) : rollPID;
      *pitch = abs(pitchPID) > PITCH_PID_MAX ? PITCH_PID_MAX*sign(pitchPID) : pitchPID;
      *yaw = abs(yawPID) > YAW_PID_MAX ? YAW_PID_MAX*sign(yawPID) : yawPID;

    }

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
    void getMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3, double roll, double pitch, double yaw) {
      uint16_t rollDelta = abs((roll/ROLL_PID_MAX) * 65535);
      boolean rollPositive = sign(roll) == -1 ? 0 :  1;
      
      uint16_t pitchDelta = abs((pitch/PITCH_PID_MAX) * 65535);
      boolean pitchPositive = sign(pitch) == -1 ? 0 :  1;

      uint16_t yawDelta = abs((yaw/YAW_PID_MAX) * 65535);
      boolean yawPositive = sign(yaw) == -1 ? 0 :  1;
      
      uint16_t highRoll, lowRoll, highPitch, lowPitch, highYaw, lowYaw;
      
      getDeltas(&highRoll, &lowRoll, rollDelta/2, roll);
      getDeltas(&highPitch, &lowPitch, pitchDelta/2, pitch);
      getDeltas(&highYaw, &lowYaw, yawDelta/2, yaw);

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
      
//      Serial.print("a:");
//      Serial.print(m0temp);
//      Serial.print("\tb:");
//      Serial.print(m1temp);
//      Serial.print("\tc:");
//      Serial.print(m2temp);
//      Serial.print("\td:");
//      Serial.println(m3temp);     

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

    void getDeltas(uint16_t *high, uint16_t *low, uint16_t deltaHalf, double pidVal) {
      if (setpoint_thr > 65535 - deltaHalf) {
        *high = 65535;
        *low = setpoint_thr - (deltaHalf + deltaHalf - (65535 - setpoint_thr));
      } else if (setpoint_thr - THR_IDLE < deltaHalf) {
        *low = THR_IDLE;
        *high = setpoint_thr + deltaHalf + (deltaHalf-(setpoint_thr-THR_IDLE));
      } else {
        *high = setpoint_thr + deltaHalf;
        *low = setpoint_thr - deltaHalf;
      }
    }

    
  public:
    PID_Controller(IMU6050 *imuRef) {
      this->imu = imuRef;
      //this->recvr = recvrRef;
    };

    void updateSetpoints(double r, double p, double y, uint16_t thr) {
      setpoint_roll = r;
      setpoint_pitch = p;
      setpoint_yaw = y;
      if (thr < THR_IDLE) {
        setpoint_thr = THR_IDLE;
      } else {
        setpoint_thr = thr;
      } 
    }

    void updateMotorValues(uint16_t *m0, uint16_t *m1, uint16_t *m2, uint16_t *m3) {
      //recvr->getSetpoints(&setpoint_roll, &setpoint_pitch, &setpoint_yaw, &setpoint_thr);
      double r, p, y;
      getPID(&r, &p, &y);
      getMotorValues(m0, m1, m2, m3, r, p, y);
    }  
};



IntervalTimer heartBeat;

boolean IS_ARMED = false;
boolean IS_FAILSAFED = true;
#define M0_PIN 14
#define M1_PIN 15
#define M2_PIN 22
#define M3_PIN 23
IMU6050 *imu; 
PID_Controller *pid; 
Receiver *recvr;
Servo MOTOR0;
Servo MOTOR1;
Servo MOTOR2;
Servo MOTOR3;

void setup() {

  
  recvr = new Receiver(true); //something is sketch with serial of this..
  MOTOR0.attach(M0_PIN);
  MOTOR1.attach(M1_PIN);
  MOTOR2.attach(M2_PIN);
  MOTOR3.attach(M3_PIN);
  writeToMotors(0,0,0,0);
  //Serial.begin(115200);
  imu = new IMU6050(25);
  pid = new PID_Controller(imu);
  delay(5000);
  //heartBeat.begin(updateAll, 1000);
  
  
}
int callTimeMillis = millis();
void loop() {
  //callTimeMillis = millis();
  //if (millis() > callTimeMillis) {
    
    updateAll();
    
    //Serial.println(micros() - start);
  //}
  };

int printTime = millis();
void updateAll() {
  double setpoint_roll = 0; //deg/s
  double setpoint_pitch = 0; //deg/s
  double setpoint_yaw = 0; //deg/s
  uint16_t setpoint_thr = 0;
  
  recvr->getSetpoints(&setpoint_roll, &setpoint_pitch, &setpoint_yaw, &setpoint_thr);
  
  IS_FAILSAFED = recvr->getFailsafe();
  IS_ARMED = recvr->getArm();
  
  pid->updateSetpoints(setpoint_roll, setpoint_pitch, setpoint_yaw, setpoint_thr);
  imu->updateRotationData();
  
  uint16_t m0, m1, m2, m3;
  pid->updateMotorValues(&m0, &m1, &m2, &m3);
  if (IS_ARMED && !IS_FAILSAFED) {
    //int start = micros();
    writeToMotors(m0, m1, m2, m3);
    //Serial.println(micros() - start);
  } else {
    writeToMotors(0,0,0,0);
  }

//  if (millis() - printTime > 20) {
//      Serial.print("a:");
//      Serial.print(m0);
//      Serial.print("\tb:");
//      Serial.print(m1);
//      Serial.print("\tc:");
//      Serial.print(m2);
//      Serial.print("\td:");
//      Serial.println(m3);
////      Serial.print("r:");
////      Serial.print(setpoint_roll);
////      Serial.print("\t");
////    
////      Serial.print("p:");
////      Serial.print(setpoint_pitch);
////      Serial.print("\t");
////    
////      Serial.print("y:");
////      Serial.print(setpoint_yaw);
////      Serial.print("\t");
////    
////      Serial.print("thr:");
////      Serial.print(setpoint_thr);
////      Serial.print("\n");
//      printTime = millis();
//  }

//  if (IS_ARMED) {
//    writeToMotors(m0, m1, m2, m3);
//  } else {
//    writeToMotors(0, 0, 0, 0);
//  }
//  Serial.print("r:");
//  Serial.print(setpoint_roll);
//  Serial.print("\t");
//
//  Serial.print("p:");
//  Serial.print(setpoint_pitch);
//  Serial.print("\t");
//
//  Serial.print("y:");
//  Serial.print(setpoint_yaw);
//  Serial.print("\t");
//
//  Serial.print("thr:");
//  Serial.print(setpoint_thr);
//  Serial.print("\n");

  
//  Serial.print(imu.angularVelocityToString());
//  Serial.print(imu.angularAccelerationToString());

//  float rollABuff[10000];
//  float rollVBuff[10000];
//  for (int i = 0; i < 10000; i++) {
//    delay(2);
//    imu.updateRotationData();
//    rollABuff[i] = imu.getRollA();
//    rollVBuff[i] = imu.getRollV();
//  }
//  for (int i = 0; i < 10000; i++) {
//    delay(1);
//    Serial.print("FP: ");
//    Serial.println(imu.getYawV());
//    Serial.print("\tRP: ");
//    Serial.print(imu.getPitchARaw());
//    Serial.print("\tFR: ");
//    Serial.print(imu.getRollA());
//    Serial.print("\tRR: ");
//    Serial.print(imu.getRollARaw());
//    Serial.print("\tFY: ");
//    Serial.print(imu.getYawA());
//    Serial.print("\tRPY: ");
//    Serial.println(imu.getRollVRaw());

//    if (abs(imu.getRollA()) > wr) {
//      wr = imu.getRollA();
//      Serial.println(wr);
//    }
//  }

//  double roll;
//  double pitch;
//  double yaw;
//  pid.getPID(&roll, &pitch, &yaw);
//  Serial.print("r: ");
//  Serial.println(yaw);
//  Serial.print("\tp: ");
//  Serial.print(pitch);
//  Serial.print("\ty: ");
//  Serial.println(yaw);
  
}

void writeToMotors(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3) {
  MOTOR0.writeMicroseconds(int(m0*1000/65536 + 1000));
  MOTOR1.writeMicroseconds(int(m1*1000/65536 + 1000));
  MOTOR2.writeMicroseconds(int(m2*1000/65536 + 1000));
  MOTOR3.writeMicroseconds(int(m3*1000/65536 + 1000));
}
