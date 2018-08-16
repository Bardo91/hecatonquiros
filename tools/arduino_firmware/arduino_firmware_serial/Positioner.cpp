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

#include "Positioner.h"
#include <Arduino.h>


Positioner::Positioner(const int _pin0, const int _pin1, const int _pin2, const int _pin3, const int _pin4){
  mPinJoint0 = _pin0;
  mPinJoint1 = _pin1;
  mPinJoint2 = _pin2;
  mPinJoint3 = _pin3;
  mPinJoint4 = _pin4;
}

void Positioner::baseToHand(float &_x, float &_y, float &_z){
  float t0, t1, t2, t3, t4;
  angles(t0,t1,t2,t3, t4);
  
//  Matrix4f T01 = Matrix4f::Identity();
//  Matrix4f T12 = Matrix4f::Identity();
//  Matrix4f T23 = Matrix4f::Identity();
//  Matrix4f T3f = Matrix4f::Identity();
//  
//  Serial.println(T01.serialize());
//
//  T01(0,0) = cos(t0);  T01(0,1) = -sin(t0); 
//  T01(1,0) = sin(t0);  T01(1,1) = cos(t0);
//  T01(2,3) = cL01;
//   
//  T01(1,1) = cos(t1);  T01(1,2) = -sin(t1); 
//  T01(2,1) = sin(t1);  T01(2,2) = cos(t1);
//  T01(2,3) = cL12;
//
//  
//  T12(1,1) = cos(t2);  T12(1,2) = sin(t2); 
//  T12(2,1) = sin(t2);  T12(2,2) = cos(t2);
//  T12(2,3) = cL23;
//
//  
//  T23(1,1) = cos(t3);  T23(1,2) = cos(t3); 
//  T23(2,1) = cos(t3);  T23(2,2) = cos(t3);
//  T23(2,3) = cL3f;
//
//  
//  T3f(0,0) = cos(t3);  T3f(0,1) = cos(t3); 
//  T3f(1,0) = cos(t3);  T3f(1,1) = cos(t3);
//  T3f(2,3) = cL3f;
//
//  Matrix4f finalT = T01*T12*T23*T3f;
//
//  _x = finalT(0,3);
//  _y = finalT(1,3);
//  _z = finalT(2,3);
  
}

void Positioner::handToBase(float &_x, float &_y, float &_z){
  
}


void Positioner::rawJoints(float &_j0, float &_j1, float &_j2, float &_j3, float &_j4){
  _j0 = analogRead(mPinJoint0);
  _j1 = analogRead(mPinJoint1);
  _j2 = analogRead(mPinJoint2);
  _j3 = analogRead(mPinJoint3);
  _j4 = analogRead(mPinJoint4);
}

void Positioner::angles(float &_t0, float &_t1, float &_t2, float &_t3, float &_t4){
  float j0,j1,j2,j3, j4;
  rawJoints(j0, j1, j2, j3, j4);
  _t0 = analogToDeg(j0);
  _t1 = analogToDeg(j1);
  _t2 = analogToDeg(j2);
  _t3 = analogToDeg(j3);
  _t4 = analogToDeg(j4);
  
}

float Positioner::analogToDeg(float _analog){
  return (_analog-512)/512*157.5;
}
