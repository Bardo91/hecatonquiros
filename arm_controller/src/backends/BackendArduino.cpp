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
#include <hecatonquiros/backends/BackendArduino.h>
#include <iostream>

namespace hecatonquiros{
    //-----------------------------------------------------------------------------------------------------------------
    bool BackendArduino::init(const Config &_config){
        if(_config.sharedSerialPort == nullptr){
            mPort = _config.port;
            mBaudRate = _config.baudrate;
            mSerialPort = new serial::Serial(mPort, mBaudRate, serial::Timeout::simpleTimeout(1000));
            mArmId = _config.armId;
            return true;
        }else{
            mSerialPort = _config.sharedSerialPort;
            mArmId = _config.armId;
            return true;
        }
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendArduino::pose(const Eigen::Matrix4f &_pose, bool _blocking){
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendArduino::joints(std::vector<float> &_joints, bool _blocking){
        if(mSerialPort != nullptr && mSerialPort->isOpen()){
            std::stringstream cmd;
            cmd << "a"<< mArmId;
            for(unsigned i = 0; i < _joints.size(); i++){
                if(i != _joints.size()-1){
                    cmd << _joints[i]*180.0/M_PI << ",";
                }else{
                    cmd << _joints[i]*180.0/M_PI << "\r\n";
                }
            } 
            mSerialPort->write(cmd.str());
            return true;
        }else{
            return false;
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendArduino::claw(const int _action, bool _blocking){
        if(mSerialPort != nullptr && mSerialPort->isOpen()){
            std::stringstream cmd;
            switch(_action){
            case 0:
                cmd << "c"<< mArmId << "c\r\n";
                break;
            case 1:
                cmd << "c"<< mArmId << "o\r\n";
                break;
            case 2:
                cmd << "c"<< mArmId << "s\r\n";
                break;
            default:
                return false;
            }
            mSerialPort->write(cmd.str());
            return true;
        }else{
            return false;
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendArduino::jointPos(const int _id){
        return 0;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendArduino::jointLoad(const int _id){
        return 0;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendArduino::jointTorque(const int _id, const bool _enable){
        return 0;
    }

}