//
//
//
//
//


#include "Arm.h"
#include <Arduino.h> 

void Arm::setup(int _pin0, int _pin1, int _pin2, int _pin3, int _pin4){
  mServo0.attach(_pin0);
  mServo1.attach(_pin1);
  mServo2.attach(_pin2);
  mServo3.attach(_pin3);
  mServo4.attach(_pin4);
  
  //joints(0, 0, 0, 0, 0);
}

void Arm::offsets(float _o0, float _o1, float _o2, float _o3, float _o4){
  mOffset0 = _o0;
  mOffset1 = _o1;
  mOffset2 = _o2;
  mOffset3 = _o3;
  mOffset4 = _o4;
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
    mTimeOpen1 = millis();
    while(mTimeOpen1+mInterval > mTimeOpen2){
      mServo4.write( 0, 255, false); 
      mTimeOpen2 = millis();
    }
    mTimeOpen1 = 0;
    mTimeOpen2 = 0;
    stopGripper();
}

void Arm::closeGripper(){
  mTimeClose1 = millis();
  while(mTimeClose1+mInterval > mTimeClose2){
    mServo4.write( 180, 255, false); 
    mTimeClose2 = millis();
  }
  mTimeClose1 = 0;
  mTimeClose2 = 0;
  
  mServo4.write(mOffset4+5);
}

void Arm::stopGripper(){
  mServo4.write(mOffset4, 40);
}


