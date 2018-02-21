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
#include <cassert>

namespace hecatonquiros{
    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const Backend::Config &_config) {
        mBackend = Backend::create(_config);
        mArmId = _config.armId;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::home(){
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::joints(std::vector<float> _q) {
        // assert(_q.size() == 3 || _q.size() == 4);

        // mArmjoints[0] = _q[0];
        // mArmjoints[1] = _q[1];
        // mArmjoints[2] = _q[2];
        // if(_q.size() == 3){
        //     mArmjoints[3] = 0;
        // }else{
        //     mArmjoints[3] = _q[3];
        // }
        
        // if(mBackend != nullptr){
        //     mBackend->joints(mArmjoints);
        // }
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<float> Arm4DoF::joints() const {
        return {0,0,0};
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::position(Eigen::Vector3f _position, float _wirst) {
        
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f Arm4DoF::position() {
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position, std::vector<float> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::directKinematic(const std::vector<float> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f Arm4DoF::pose() const {
        return Eigen::Matrix4f();
    }
    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::lastTransformations(Eigen::Matrix4f &_t0, Eigen::Matrix4f &_t1, Eigen::Matrix4f &_t2) {
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

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::isInit() const{
        return mBackend != nullptr;
    }
}