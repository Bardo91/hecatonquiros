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
#include <arm_controller/model_solvers/ModelSolverSimple4Dof.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cassert>

namespace hecatonquiros{
    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const ModelSolver::Config &_modelConfig, const Backend::Config &_backendConfig) {
        mBackend = Backend::create(_backendConfig);
        mArmId = _backendConfig.armId;
        mModelSolver = ModelSolver::create(_modelConfig);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::home(){
        mModelSolver->joints({mHome1, mHome2, mHome3, mHome4});
        mBackend->joints({mHome1, mHome2, mHome3, mHome4});
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::joints(std::vector<float> _q) {
        mModelSolver->joints(_q);
        
        if(mBackend != nullptr){
            mBackend->joints(_q);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<float> Arm4DoF::joints() const {
        return mModelSolver->joints();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::position(Eigen::Vector3f _position, float _wirst) {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,1>(0,3) = _position;
        std::vector<float> angles;
        if(mModelSolver->checkIk(pose, angles, false)){
            angles[3] = _wirst; //
            mBackend->joints(angles);
            mModelSolver->joints(angles);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f Arm4DoF::position() {
        std::vector<Eigen::Matrix4f> transforms;
        mModelSolver->jointsTransform(transforms);
        return transforms.back().block<3,1>(0,3);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position, std::vector<float> &_angles){
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,1>(0,3) = _position;
        return mModelSolver->checkIk(pose, _angles, false);
    }


    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _pose, std::vector<float> &_angles, bool _forceOri){
        return mModelSolver->checkIk(_pose, _angles, _forceOri);
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f Arm4DoF::pose() const {
        std::vector<Eigen::Matrix4f> transforms;
        mModelSolver->jointsTransform(transforms);
        return transforms.back();
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