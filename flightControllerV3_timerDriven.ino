#include <Servo.h>
#include "PIDV3.h"
#include "IMU6050_.h"
//#include "PID.h"
#include "Receiver.h"
//#include "PIDv2.h"
//#include "TeensyTimerTool.h"
//using namespace TeensyTimerTool;

//IntervalTimer timingsCall;
//IntervalTimer loopCall;

#define M0_PIN 15
#define M1_PIN 14
#define M2_PIN 22
#define M3_PIN 23
#define CALL_PRD_US 999
static IMU6050_ *imu;
 
static PIDV3 *pid; 
static Receiver *recvr;
static Servo MOTOR0;
static Servo MOTOR1;
static Servo MOTOR2;
static Servo MOTOR3;
void setup() {
  Serial.begin(115200);
  recvr = new Receiver(true); //something is sketch with serial of this..
  MOTOR0.attach(M0_PIN);
  MOTOR1.attach(M1_PIN);
  MOTOR2.attach(M2_PIN);
  MOTOR3.attach(M3_PIN);
  writeToMotors(0,0,0,0, false);
  imu = new IMU6050_(40, 0);
  pid = new PIDV3(imu, recvr);
  delay(1000);
  //Serial.print("done");
  
//  timingsCall.begin(testFunc, 100);
//  loopCall.begin(testFunc_, 1000);
  
  
}

unsigned long prevCallTime = 0;
unsigned long split = 0;
void loop() {
  split = micros() - prevCallTime;
  if (split > CALL_PRD_US) {
    prevCallTime = micros();
    //Serial.println(split);
    updateAll();
  }
};
  

void updateAll() {
    imu->updateRotationDataV2();
    Serial.print(imu->angularVelocityToString());
    unsigned int m0, m1, m2, m3;
    pid->getMotorValues(&m0, &m1, &m2, &m3, false);
    writeToMotors(m0, m1, m2, m3, false);
}

void writeToMotors(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3, boolean toString) {
  MOTOR0.writeMicroseconds(m0);
  MOTOR1.writeMicroseconds(m1);
  MOTOR2.writeMicroseconds(m2);
  MOTOR3.writeMicroseconds(m3);
  if (toString) {
      Serial.print("a:" + String(m0) + "\t");
      Serial.print("b:" + String(m1) + "\t");
      Serial.print("c:" + String(m2) + "\t");
      Serial.print("d:" + String(m3) + "\n");
  }
}
