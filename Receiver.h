//#include "sbus.h"
//class Receiver {
//  private:
//    const uint16_t CH_MAX = 1811;
//    const uint16_t CH_MIN = 172;
//    const uint16_t CH_MID = 992;
//    #define FAILSAFE 9
//    #define THR 2
//    #define YAW 3
//    #define PITCH 1
//    #define ROLL 0
//    #define ARM 4 //aux switch
//    #define TURTLE 5
//    #define BEEPER 7
//    #define RX_PORT Serial8
//    bfs:: SbusRx *sbus;
//    bfs::SbusData *data_;
//    double roll_rate = 0;
//    double pitch_rate = 0;
//    double yaw_rate = 0;
//    double thr_rate = 0;
//    uint16_t arm_stick = CH_MID;
//    uint16_t turtle_stick = CH_MID;
//    uint16_t beeper_stick = CH_MID;
//    uint16_t failsafe_stat = 0;
//    const double CUBIC_AMP =  0.00000117956886463;
//    const double LINEAR_AMP = 0.791208791209;
//    
//    double rates(uint16_t rawCh);
//    uint16_t convertThr(uint16_t thr);
//    boolean update_rates();
//  
//  public:
//    Receiver(boolean invert);
//    void getSetpoints(double *r, double *p, double *y, uint16_t *thr);
//    boolean getArm();
//    boolean getFailsafe();
//};

#include "sbus.h"
class Receiver {
  private:
    bfs:: SbusRx *sbus;
    bfs::SbusData data_;
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
    
    double rates(uint16_t rawCh);
//    {
//      double midPtOffset = ((double)rawCh - (double)CH_MID);
//      return ((CUBIC_AMP*pow(midPtOffset, 3) + LINEAR_AMP*(midPtOffset))/2.0);
//    }
    
    uint16_t convertThr(uint16_t thr); 
//    {
//      return (thr - CH_MIN)*65535/CH_MAX;
//    }

    boolean update_rates();
//    {
//      if(sbus->Read()){
//        *data_ = sbus->data();
//        //Serial.println(this->data_->ch[ROLL]);
//        roll_rate = rates(data_->ch[ROLL]);
//        pitch_rate = rates(data_->ch[PITCH]);
//        yaw_rate = rates(data_->ch[YAW]);
//        thr_rate = convertThr(data_->ch[THR]);
//        arm_stick = data_->ch[ARM];
//        turtle_stick = data_->ch[TURTLE];
//        beeper_stick = data_->ch[BEEPER];
//        failsafe_stat = data_->ch[FAILSAFE];
//        return true;
//      }  
//      return false;
//      
//  }
  public:
    Receiver(boolean invert); 
//    {
//      sbus = new bfs::SbusRx(&Serial8); //(serialPort, inverted, highspeed)
//      data_ = &sbus->data();
//      sbus->Begin();
//    }
    void getSetpoints(double *r, double *p, double *y, uint16_t *thr);
//    {
//      update_rates();
//      *r = roll_rate;
//      *p = pitch_rate;
//      *y = yaw_rate;
//      *thr = thr_rate;
//    }
    boolean getArm(); 
//    {
//      return arm_stick > CH_MID + 200;
//    }
    boolean getFailsafe();
//    {
//      return failsafe_stat == 0;
//    }
    boolean test();

};
