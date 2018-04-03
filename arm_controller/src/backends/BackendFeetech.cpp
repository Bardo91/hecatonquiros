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
#include <chrono>

namespace hecatonquiros{
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::init(const Config &_config){
        changeValues(_config.valuesMinMax);
        mOffsetJoints = _config.jointsOffsets;
        mSerialPort = _config.port;
        mArmId = _config.armId;
        mServoDriver =  new SCServo(mSerialPort);
        if(mServoDriver->isConnected()){
            mLoadChecker = std::thread([&](){
                while(mServoDriver->isConnected()){
                    //for(auto &id:mUsedJoints){
                    //    int load = mServoDriver->ReadLoadH(mArmId*10 + id + 1);
                    //    if(load > 200)
                    //        std::cout << "WARNING: Load of servo " << mArmId*10 + id + 1 << " is " << load <<std::endl;
                    //    // 666 TODO do something with it;
                    //}
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            });
            return true;
        }else{
            return false;
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::pose(const Eigen::Matrix4f &_pose, bool _blocking){
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::joints(const std::vector<float> &_joints, bool _blocking){
        if(_joints.size() > mOffsetJoints.size()){
            for(int i = mOffsetJoints.size() ; i<_joints.size(); i++){
                mOffsetJoints.push_back(0); // Fill with zeros
            }
        }
        if(_joints.size() > mUsedJoints.size()){
            for(int i = mUsedJoints.size() ; i<_joints.size(); i++){
                mUsedJoints.push_back(i);
            }
        }
        if(mServoDriver->isConnected()){
            for(unsigned i = 0; i < _joints.size(); i++){
                mServoDriver->WritePos(mArmId*10 + i + 1, mapAngleToVal(mMinMaxValues[i].first, mMinMaxValues[i].second, _joints[i] + mOffsetJoints[i]), mSpeed);
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
    bool BackendFeetech::changeValues(std::vector<std::pair<float, float> >_newvalues){
        for(unsigned i = 0; i < _newvalues.size(); i++){
            mMinMaxValues[i].first = _newvalues[i].first/180.0*M_PI;
            mMinMaxValues[i].second = _newvalues[i].second/180.0*M_PI;
        }
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetech::mapAngleToVal(float _minAngle, float _maxAngle, float _angle){
        return (_angle-_minAngle)/(_maxAngle-_minAngle)*(1023-0);
    }
}