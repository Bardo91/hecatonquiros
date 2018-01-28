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


#include <arm_controller/Arm4DoF.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
namespace hecatonquiros{
    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const Backend::Config &_config) {
        mBackend = Backend::create(_config);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::home(){
    joints({mHome1, mHome2, mHome3});
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::joints(std::vector<float> _q) {
        mArmjoints[0] = _q[0];
        mArmjoints[1] = _q[1];
        mArmjoints[2] = _q[2];

        if(mBackend != nullptr){
            mBackend->joints(mArmjoints);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<float> Arm4DoF::joints() const {
        return mArmjoints;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::position(Eigen::Vector3f _position) {
        mArmjoints[0] = atan2(_position[1], _position[0]);
    
        float y = sqrt(_position[0]*_position[0] + _position[1]*_position[1]);
        float x = _position[2] - mBaseHeight;
    
        float D = (x*x+y*y-mHumerus*mHumerus-mRadius*mRadius)/(2*mHumerus*mRadius);
        float auxD = 1 - D*D;
        if(auxD < 0){
            auxD = 0;
            //std::cout << "1-D^2 < 0, unrecheable point, fitting to 0\n";
        }
    
        float theta2a = atan2(sqrt(auxD), D);
        float theta2b = atan2(-sqrt(auxD), D);
    
        float k1a = mHumerus + mRadius*cos(theta2a);
        float k2a = mRadius*sin(theta2a);
        float theta1a = atan2(y,x)-atan2(k2a,k1a);
    
        float k1b = mHumerus + mRadius*cos(theta2b);
        float k2b = mRadius*sin(theta2b);
        float theta1b = atan2(y,x)-atan2(k2b,k1b);
    
        float angleDistA = abs(theta1a - mArmjoints[1]) + abs(theta2a - mArmjoints[2]);
        float angleDistB = abs(theta1b - mArmjoints[1]) + abs(theta2b - mArmjoints[2]);
    
        //if(angleDistA < angleDistB){
            mArmjoints[1] = theta1a;
            mArmjoints[2] = theta2a;
        //}else{
        //    mArmjoints[1] = theta1b;
        //    mArmjoints[2] = theta2b;
        //}
    
        joints(mArmjoints); // send joints to arduino
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f Arm4DoF::position() {
        float mA0 = mArmjoints[0];
        float mA1 = mArmjoints[1];
        float mA2 = mArmjoints[2];

        mT01 <<	cos(mA0),   0,  sin(mA0),  0,
                sin(mA0),   0,  -cos(mA0), 0,
                0,          1,  0,         mBaseHeight,
                0,          0,  0,         1;

        mT12 <<	cos(M_PI/2 + mA1),  -sin(M_PI/2 + mA1),  0,  mHumerus * cos(M_PI/2 + mA1),
                sin(M_PI/2 + mA1),  cos(M_PI/2 + mA1),  0,  mHumerus * sin(M_PI/2 + mA1),
                0,                  0,                  1,  0,
                0,                  0,                  0,  1;

        mT23 <<	cos(mA2),  -sin(mA2),  0,   mRadius * cos(mA2),
                sin(mA2),  cos(mA2),   0,   mRadius * sin(mA2),
                0,         0,          1,   0,
                0,         0,          0,   1;

        mFinalT = mT01*mT12*mT23;

        return mFinalT.block<3,1>(0,3);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position){
        std::vector<Eigen::Matrix4f> ts;
        std::vector<float> angles;
        return checkIk(_position, angles, ts);
    }


    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position, std::vector<Eigen::Matrix4f> &_transformations){
        std::vector<float> angles;
        return checkIk(_position, angles,_transformations);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position, std::vector<float> &_angles){
        std::vector<Eigen::Matrix4f> ts;
        return checkIk(_position,_angles, ts);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position, std::vector<float> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        float theta0a = atan2(_position[1], _position[0]);
    
        float y = sqrt(_position[0]*_position[0] + _position[1]*_position[1]);
        float x = _position[2] - mBaseHeight;
    
        float D = (x*x+y*y-mHumerus*mHumerus-mRadius*mRadius)/(2*mHumerus*mRadius);
        float auxD = 1 - D*D;
        if(auxD < 0){ // NON REACHABLE POINT
            //std::cout << "1-D^2 < 0, unrecheable point, fitting to 0\n";
            return false;
        }else{  // REACHABLE POINT
            float theta2a = atan2(sqrt(auxD), D);

            float k1a = mHumerus + mRadius*cos(theta2a);
            float k2a = mRadius*sin(theta2a);
            float theta1a = atan2(y,x)-atan2(k2a,k1a);
            _angles = {theta0a, theta1a, theta2a};

            return directKinematic(_angles, _transformations);
        } 
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::directKinematic(const std::vector<float> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        Eigen::Matrix4f t01, t12, t23;

        t01 <<	cos(_angles[0]),  -sin(_angles[0]),  0, 0,
                sin(_angles[0]),  cos(_angles[0]), 0, 0,
                0,          0,  1,         mBaseHeight,
                0,          0,  0,         1;

        t12 <<	cos(-M_PI/2 + _angles[1]), 0, sin(-M_PI/2 + _angles[1]), mHumerus * cos(-M_PI/2 + _angles[1]),
                0,1,0,0,
                -sin(-M_PI/2 + _angles[1]), 0, cos(-M_PI/2 + _angles[1]),  -mHumerus * sin(-M_PI/2 + _angles[1]),
                0,                  0,                  0,  1;

        t23 <<	cos(_angles[2]),  0, sin(_angles[2]),   mRadius * cos(_angles[2]),
                0,1,0,0,
                -sin(_angles[2]), 0,  cos(_angles[2]),    -mRadius * sin(_angles[2]),
                0,         0,          0,   1;


        _transformations.push_back(t01);
        _transformations.push_back(t12);
        _transformations.push_back(t23);

        if( (_angles[0]>=(-90*M_PI/180) && _angles[0]<=(90*M_PI/180)) && 
            (_angles[1]>=(-90*M_PI/180) && _angles[1]<=(90*M_PI/180)) && 
            (_angles[2]>=(-90*M_PI/180) && _angles[2]<=(90*M_PI/180)) ){
            // Perfectly reachable point
            return true;
        }
        else{   
            // Exceeding some joint max angle
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f Arm4DoF::pose() const {
        return Eigen::Matrix4f();
    }
    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::lastTransformations(Eigen::Matrix4f &_t0, Eigen::Matrix4f &_t1, Eigen::Matrix4f &_t2) {
        position();
        _t0 = mT01;
        _t1 = mT12;
        _t2 = mT23;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::closeClaw(){
        if(mBackend != nullptr){
            mBackend->claw(0);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::openClaw(){
        if(mBackend != nullptr){
            mBackend->claw(2);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::stopClaw(){
        if(mBackend != nullptr){
            mBackend->claw(1);
        }
    }
}