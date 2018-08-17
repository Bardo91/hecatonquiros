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


#include "Matrix.h"

class Positioner{
public:
  Positioner(const int _pin0, const int _pin1, const int _pin2, const int _pin3, const int _pin4);

  void baseToHand (float &_x, float &_y, float &_z);
  void handToBase (float &_x, float &_y, float &_z);
  void rawJoints  (float &_j0, float &_j1, float &_j2, float &_j3, float &_j4);
  void angles     (float &_t0, float &_t1, float &_t2, float &_t3, float &_t4);
private:
  float analogToDeg(float _analog);

  
private:
  int mPinJoint0;
  int mPinJoint1;
  int mPinJoint2;
  int mPinJoint3;
  int mPinJoint4;

  const float cL01 = 68.0;
  const float cL12 = 169.0;
  const float cL23 = 169.0;
  const float cL34 = 112.0;
  const float cL4f = 70.0;
};

