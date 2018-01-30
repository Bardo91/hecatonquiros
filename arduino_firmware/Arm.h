//
//
//
//
//

#include "VarSpeedServo.h"

class Arm{
  public:
    void setup(int _pin0, int _pin1, int _pin2, int _pin3, int _pin4);

    void offsets(float _o0, float _o1, float _o2, float _o3, float _o4);

    void speed(float _speed);
    float speed();

    void joints(float _v0, float _v1, float _v2);

    void wrist(float angle);

    void openGripper();

    void closeGripper();

    void stopGripper();

  private:
    VarSpeedServo mServo0, mServo1, mServo2, mServo3, mServo4;
    float mOffset0=90, mOffset1=90, mOffset2=90, mOffset3=90, mOffset4=90;
    float mSpeed=10;
    int mFlagClose = 0;

    unsigned long mTimeClose1, mTimeClose2, mTimeOpen1, mTimeOpen2 = 0;
    unsigned long mInterval = 700;
};

