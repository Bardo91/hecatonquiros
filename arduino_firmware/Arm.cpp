//
//
//
//
//


#include "Arm.h"
#include <Arduino.h> 

void Arm::setup(int _pin0, int _pin1, int _pin2, int _pin3, int _pin4, int _pin5){
  mServo0.attach(_pin0);
  mServo1.attach(_pin1);
  mServo2.attach(_pin2);
  mServo3.attach(_pin3);
  mServo4.attach(_pin4);
  
  mPinSenFuerza = _pin5;
  
  //joints(0, 0, 0, 0, 0);
}

void Arm::offsets(float _o0, float _o1, float _o2, float _o3, float _o4, float _o5){
  mOffset0 = _o0;
  mOffset1 = _o1;
  mOffset2 = _o2;
  mOffset3 = _o3;
  mOffset4 = _o4;
  mTresholdSF = _o5;
}

void Arm::speed(float _speed){
  mSpeed = _speed;
}


float Arm::speed(){
  return mSpeed;
}

void Arm::joints(float _v0, float _v1, float _v2){
  mServo0.write(_v0 + mOffset0, mSpeed, false);
  mServo1.write(_v1 + mOffset1, mSpeed, false);
  mServo2.write(_v2 + mOffset2, mSpeed, false); 
}

void Arm::wrist(float _angle){
  mServo3.write(_angle + mOffset3, mSpeed, false);  
}

void Arm::openGripper(){
  if(mFlagClose){
    mFlagClose = 0;
    //mTimeOpen1 = mTimeClose2 - mTimeClose1;
    //while(mTimeOpen1 > 1){
    mTimeOpen1 = millis();
    while(mTimeOpen1+mInterval > mTimeOpen2){
      mServo4.write(0, 0);
      mTimeOpen2 = millis();
      //mTimeOpen1--;
    }
    stopGripper();
  }
}

void Arm::closeGripper(){
  //mTimeClose1 = millis();
  while(mSenFuerza <= mTresholdSF){
    mServo4.write(180, 0);
    mSenFuerza = analogRead(mPinSenFuerza);
    //Serial.println(mSenFuerza);
  }
  //mTimeClose2 = millis();
  mSenFuerza = 0;
  mServo4.write(mOffset4+5);
  mFlagClose = 1;
}

void Arm::stopGripper(){
  mServo4.write(mOffset4, 40);
}


