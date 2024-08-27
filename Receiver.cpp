#include "Receiver.h"
#include "sbus.h"

Receiver::Receiver(boolean invert) 
    {
      sbus = new bfs::SbusRx(port); //(serialPort, inverted, highspeed)
      data_ = sbus->data();
      sbus->Begin();
    }
//convert raw channel input to deg/s value given equation constants 
float Receiver::rates(uint16_t rawCh) 
    {
      float midPtOffset = ((float)rawCh - (float)CH_MID);
      float dps = ((CUBIC_AMP*pow(midPtOffset, 3) + LINEAR_AMP*(midPtOffset))/2.0);
      //if (abs(dps) < 10) dps = 0;
      return dps; //NEW CHANGE
    }
//converts throttle to 16 bit resolution value
uint16_t Receiver::convertThr(uint16_t thr) 
    {
      return (thr - CH_MIN)*1000/CH_MAX + 1000;
    }
//updates all channels to store in member vars
boolean Receiver::update_rates() 
    {
      if(sbus->Read()){
        data_ = sbus->data();
        //Serial.println(this->data_->ch[ROLL]);
        roll_rate = rates(data_.ch[ROLL]);
        pitch_rate = rates(data_.ch[PITCH]);
        yaw_rate = rates(data_.ch[YAW]);
        thr_rate = convertThr(data_.ch[THR]);
        arm_stick = data_.ch[ARM];
        turtle_stick = data_.ch[TURTLE];
        beeper_stick = data_.ch[BEEPER];
        failsafe_stat = data_.ch[FAILSAFE];
        return true;
      }  
      return false;
      
  }
//asigns inoutted var addrs to current input deg/s values
void Receiver::getSetpoints(float *r, float *p, float *y, unsigned int *thr) 
    {
      update_rates();
      *r = roll_rate;
      *p = pitch_rate;
      *y = yaw_rate;
      *thr = thr_rate;
    }
//get arm status
boolean Receiver::getArm() {
  return arm_stick > CH_MID + 200;
}
//get if failsafe
boolean Receiver::getFailsafe() {
  return failsafe_stat == 0;
}

boolean Receiver::test() {
  Serial.print("recv test");
  return true;
}
