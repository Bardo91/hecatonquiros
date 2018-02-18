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


#include <SCServo.h>

/// Class for using 4DoF arm equiped with Serial servos from FeeTech.
/// WARNING! This class assumes that the board has at least a second serial interface dedicated to the arms.
class Arm4DoF{
  public:
    /// Set id of the arm. ID of servos are computed according to this ID, e.g. ID= 3. Servo1_id=31, Servo2_id=32, etc.
    void setup(int _id);

    /// Set fine tune offsets of the joints
    /// \param _o0: offset of first servo
    /// \param _o1: offset of second servo
    /// \param _o2: offset of third servo
    /// \param _o3: offset of fourth servo
    /// \param _o4: offset of fith servo
    void offsets(float _o0, float _o1, float _o2, float _o3, float _o4);

    /// Set servo speed
    /// \param _speed: desired speed for servos.
    void speed(float _speed);

    /// Get servo speed
    /// \return servo speed
    float speed();

    /// Set target joint coordinates
    /// \param _v0: desired angle for joint 0
    /// \param _v1: desired angle for joint 1
    /// \param _v2: desired angle for joint 2
    /// \param _v3: desired angle for joint 3
    void joints(float _v0, float _v1, float _v2, float _v3);

    /// Set wirst angle
    /// \param _angle: desired angle for wirst
    void wrist(float _angle);

    /// Open gripper
    void openGripper();

    /// Close gripper
    void closeGripper();

  private:
    // Map from 0 to 1023 according to min and max angle
    int mapAngleToVal(float _minAngle, float _maxAngle, float _angle);
    
  private:
    SCServo mServosInterface;
    float mOffset0=90, mOffset1=90, mOffset2=90, mOffset3=90, mOffset4=90;
    float mSpeed=4000;
    int mFlagClose = 0;

    unsigned long mTimeClose1, mTimeClose2, mTimeOpen1, mTimeOpen2 = 0;
    unsigned long mInterval = 700;
};

