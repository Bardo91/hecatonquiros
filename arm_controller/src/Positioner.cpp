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


#include <hecatonquiros/Positioner.h>

#include <iostream>
namespace hecatonquiros{
    
    //---------------------------------------------------------------------------------------------------------------------
    Positioner::Positioner(std::string _port, int _baudrate) {
        mArduinoCom = new serial::Serial(_port, _baudrate, serial::Timeout::simpleTimeout(1000));
        init();
    }

    //---------------------------------------------------------------------------------------------------------------------
    Positioner::Positioner(serial::Serial *_serialPort) {
        mArduinoCom = _serialPort;
        init();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::init() {
        mP0 = {247.778,         486.778,  0.0};
        mP1 = {250.827,         504.296,  0.0}; 
        mP2 = {-254.328,        492.932,  0.0}; 
        mP3 = {-250.889,        521.304,  0.0}; 
        mP4 = {251.814,         523.955,  0.0};

        mSerialThread = std::thread([&]() {
            std::string data = "";
            while (mRun) {
                mSecureRead.lock();
                if(mToggleLock){
                    mArduinoCom->write("l\r\n");
                    mToggleLock = false;
                }
                mArduinoCom->write("p\r\n");
                std::string data = mArduinoCom->readline();
                try {
                    if (data.find_first_of(",") != data.npos) {
                        // Get joints   666 Make this more efficient
                        mJ4 = atof(data.substr(0, data.find_first_of(",")).c_str());
                        data = data.substr(data.find_first_of(",") + 1, data.size());
                        mJ3 = atof(data.substr(0, data.find_first_of(",")).c_str());
                        data = data.substr(data.find_first_of(",") + 1, data.size());
                        mJ2 = atof(data.substr(0, data.find_first_of(",")).c_str());
                        data = data.substr(data.find_first_of(",") + 1, data.size());
                        mJ1 = atof(data.substr(0, data.find_first_of(",")).c_str());
                        data = data.substr(data.find_first_of(",") + 1, data.size());
                        mJ0 = atof(data.substr(0, data.find_first_of(",")).c_str());
                        data = data.substr(data.find_first_of(",") + 1, data.size());
                        
                        // Get acceleration vector
                        float ax = atof(data.substr(0, data.find_first_of(",")).c_str());
                        data = data.substr(data.find_first_of(",") + 1, data.size());
                        float ay = atof(data.substr(0, data.find_first_of(",")).c_str());
                        data = data.substr(data.find_first_of(",") + 1, data.size());
                        float az = atof(data.c_str());

                        Eigen::Vector3f n = {ax,ay,az};
                        n /= n.norm();
                        mAccelerationDirection = n;
                    }
                }
                catch (int _e){
                    std::cout << "Read exception" << std::endl;
                }
                mSecureRead.unlock();
                //std::cout << mJ0 << ", " << mJ1 << ", " << mJ2 << ", " << mJ3 << ", " << mJ4 << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        std::cout << "Initialized Positioner" << std::endl;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Positioner::close() {
        mRun = false;
        while (!mSerialThread.joinable()) {}
        mSerialThread.join();

        return true;
    }

    void Positioner::toggleLock(){
        mToggleLock = true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::baseToHand (Eigen::Matrix4f &_pose){
        std::vector<float> vals = angles();
    

        mT01 <<	cos(vals[0]), 0, sin(vals[0]), 0,
                sin(vals[0]), 0, -cos(vals[0]), 0,
                0, 1, 0, cL01,
                0, 0, 0, 1;
    
        mT12 <<	cos(M_PI / 2 + vals[1]), -sin(M_PI / 2 + vals[1]), 0, (cL12 * cos(M_PI / 2 + vals[1])),
                sin(M_PI / 2 + vals[1]), cos(M_PI / 2 + vals[1]), 0, (cL12 * sin(M_PI / 2 + vals[1])),
                0, 0, 1, 0,
                0, 0, 0, 1;

        mT23<<	cos(vals[2]), -sin(vals[2]), 0, (cL23 * cos(vals[2])),
                sin(vals[2]), cos(vals[2]), 0, (cL23 * sin(vals[2])),
                0, 0, 1, 0,
                0, 0, 0, 1;

    
        mT34 <<	cos(vals[3]), 0, sin(vals[3]), (cL34 * cos(vals[3])),
                sin(vals[3]), 0, -cos(vals[3]), (cL34 * sin(vals[3])),
                0, 1, 0, 0,
                0, 0, 0, 1;

        mT4f << cos(vals[4]), -sin(vals[4]), 0, (cL4f * cos(vals[4])),
                sin(vals[4]), cos(vals[4]), 0, (cL4f * sin(vals[4])),
                0, 0, 1, 0,
                0, 0, 0, 1;

        _pose = mT01*mT12*mT23*mT34*mT4f;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::baseToHand (Eigen::Vector3f &_position){
        Eigen::Matrix4f pose;
        baseToHand(pose);
        _position = pose.block<3,1>(0,3);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::baseToHand(float &_x, float &_y, float &_z){
        Eigen::Matrix4f pose;
        baseToHand(pose);

        _x = pose(0,3);
        _y = pose(1,3);
        _z = pose(2,3);
    
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::handToBase(float &_x, float &_y, float &_z){
        Eigen::Matrix4f pose;
        handToBase(pose);

        _x = pose(0,3);
        _y = pose(1,3);
        _z = pose(2,3);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::handToBase (Eigen::Vector3f &_position){
        Eigen::Matrix4f pose;
        handToBase(pose);
        _position = pose.block<3,1>(0,3);
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::handToBase (Eigen::Matrix4f &_pose){
        Eigen::Matrix4f pose;
        baseToHand(pose);
        _pose  = pose.inverse();
    }
    

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::rawJoints(float &_j0, float &_j1, float &_j2, float &_j3, float &_j4){
        mSecureRead.lock();
        _j0 = mJ0;
        _j1 = mJ1;
        _j2 = mJ2;
        _j3 = mJ3;
        _j4 = mJ4;
        mSecureRead.unlock();
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<float> Positioner::rawJoints  (){
        std::lock_guard<std::mutex> guard(mSecureRead);
        return {mJ0, mJ1, mJ2, mJ3, mJ4};
    }
		
    //---------------------------------------------------------------------------------------------------------------------
    std::vector<float> Positioner::angles(){
        std::vector<float> vals(5);
        mSecureRead.lock();
        vals[0] = mP0.valToAngle(mJ0);
        vals[1] = mP1.valToAngle(mJ1);
        vals[2] = -mP2.valToAngle(mJ2);
        vals[3] = -mP3.valToAngle(mJ3);
        vals[4] = -mP4.valToAngle(mJ4);
        mSecureRead.unlock();
        return vals;
    }


    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f Positioner::accelerationVector(){
        return mAccelerationDirection;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Positioner::lastTransforms(Eigen::Matrix4f & _T01, Eigen::Matrix4f & _T12, Eigen::Matrix4f & _T23, Eigen::Matrix4f & _T34, Eigen::Matrix4f & _T4f) {
        _T01 = mT01;
        _T12 = mT12;
        _T23 = mT23;
        _T34 = mT34;
        _T4f = mT4f;
    }
}