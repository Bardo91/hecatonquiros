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


#include <serial/serial.h> 
#include <hecatonquiros/backends/BackendFeetech.h>
#include <iostream>

namespace hecatonquiros{
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::init(const Config &_config){
        mSerialPort = _config.port;
        mArmId = _config.armId;
        mServoDriver =  new SCServo(mSerialPort);
        return mServoDriver->isConnected();
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::pose(const Eigen::Matrix4f &_pose, bool _blocking){
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::joints(const std::vector<float> &_joints, bool _blocking){
        if(mServoDriver->isConnected()){
            for(unsigned i = 0; i < _joints.size(); i++){
                mServoDriver->WritePos(mArmId*10 + i + 1, mapAngleToVal(mMinMaxValues[i].first, mMinMaxValues[i].second, _joints[i]), mSpeed);
            }
            return true;
        }
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::claw(const int _action){
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetech::mapAngleToVal(float _minAngle, float _maxAngle, float _angle){
        return (_angle-_minAngle)/(_maxAngle-_minAngle)*(1023-0);
    }
}