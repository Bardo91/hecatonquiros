//---------------------------------------------------------------------------------------------------------------------
//  HECATONQUIROS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------



#include "Arm4DoF.h"
#include <Arduino.h> 

//--------------------------------------------------------------------------------------------------------------------
void Arm::setup(int _id){
  mId = _id;
  if(!Serial2){
    Serial2.begin(1000000)
  }
  mServosInterface.pSerial(&Serial2)
  mServosInterface.EnableTorque(_id*10 + 1, 1); // ENABLE Joint 0
  mServosInterface.EnableTorque(_id*10 + 2, 1); // ENABLE Joint 1
  mServosInterface.EnableTorque(_id*10 + 3, 1); // ENABLE Joint 2
  mServosInterface.EnableTorque(_id*10 + 4, 1); // ENABLE joint wirst
  mServosInterface.EnableTorque(_id*10 + 5, 1); // ENABLE Joint gripper
  
}

//--------------------------------------------------------------------------------------------------------------------
void Arm::offsets(float _o0, float _o1, float _o2, float _o3, float _o4){
  mOffset0 = _o0;
  mOffset1 = _o1;
  mOffset2 = _o2;
  mOffset3 = _o3;
  mOffset4 = _o4;
}

//--------------------------------------------------------------------------------------------------------------------
void Arm::speed(float _speed){
  mSpeed = _speed;
}

//--------------------------------------------------------------------------------------------------------------------
float Arm::speed(){
  return mSpeed;
}

//--------------------------------------------------------------------------------------------------------------------
void Arm::joints(float _v0, float _v1, float _v2, float _v3){
  mServosInterface.WritePos(_id*10 + 1, mapAngleToVal(-90,90,_val), mSpeed); <<--- Comprobar angulos max y min
  mServosInterface.WritePos(_id*10 + 2, mapAngleToVal(-90,90,_val), mSpeed);
  mServosInterface.WritePos(_id*10 + 3, mapAngleToVal(-90,90,_val), mSpeed);
  mServosInterface.WritePos(_id*10 + 4, mapAngleToVal(-90,90,_val), mSpeed);
  delay or sync???
}

//--------------------------------------------------------------------------------------------------------------------
void Arm::wrist(float _angle){
  mServosInterface.WritePos(_id*10 + 4, mapAngleToVal(-180,180,_val), mSpeed); <<--- Comprobar angulos max y min
}

//--------------------------------------------------------------------------------------------------------------------
void Arm::openGripper(){
  mServosInterface.WritePos(_id*10 + 5, 0, mSpeed);
}

//--------------------------------------------------------------------------------------------------------------------
void Arm::closeGripper(){
  mServosInterface.WritePos(_id*10 + 5, 1023, mSpeed);
}

//--------------------------------------------------------------------------------------------------------------------
int mapAngleToVal(float _minAngle, float _maxAngle, float _angle){
  return (_angle-_minAngle)/(_maxAngle-_minAngle)*(1023-0);
}


