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


#include <hecatonquiros/Arm4DoF.h>
#include <hecatonquiros/model_solvers/ModelSolver.h>

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
    Arm4DoF::Arm4DoF(const Backend::Config &_backendConfig) {
        mBackend = Backend::create(_backendConfig);
        mArmId = _backendConfig.armId;
        mModelSolver = nullptr;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::home(){
        std::vector<float> angles = {mHome1, mHome2, mHome3, mHome4, mHome5, mHome6};
        joints(angles, true);

        if(mModelSolver != nullptr){  /// 666 Just for visualizing transforms in joints in ModelSolverOpenRave
            std::vector<Eigen::Matrix4f> transforms;
            mModelSolver->jointsTransform(transforms);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::joints(std::vector<float> _q, bool _actuateBackend) {
        if(mModelSolver != nullptr){
            mModelSolver->joints(_q);
        }

        if(_actuateBackend){
            if(mBackend != nullptr){
                mBackend->joints(_q);
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<float> Arm4DoF::joints() const {
        if(mModelSolver != nullptr){
            return mModelSolver->joints();
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return {};
        }
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::position(Eigen::Vector3f _position, float _wirst) {
        if(mModelSolver != nullptr){
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block<3,1>(0,3) = _position;
            std::vector<float> angles;
            if(mModelSolver->checkIk(pose, angles, ModelSolver::IK_TYPE::IK_3D)){
                angles[3] = _wirst; //
                joints(angles);
            }
        }else{
            std::cout << "No model solver instantiated" << std::endl;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f Arm4DoF::position() {
        if(mModelSolver != nullptr){
            std::vector<Eigen::Matrix4f> transforms;
            mModelSolver->jointsTransform(transforms);
            return transforms.back().block<3,1>(0,3);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return Eigen::Vector3f();
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position, std::vector<float> &_angles){
        if(mModelSolver != nullptr){
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block<3,1>(0,3) = _position;
            return mModelSolver->checkIk(pose, _angles, ModelSolver::IK_TYPE::IK_3D);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::getSmoothTraj(std::vector<Eigen::Matrix4f> _pose, std::vector<std::vector<double>> &_traj, float &_time){
        if(mModelSolver != nullptr){
            return mModelSolver->getPointsTrajectory(_pose, _traj, _time); 
            return false;
        }else{
            std::cout << "No model solver OPENRAVE instantiated" << std::endl;
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _pose, std::vector<float> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
        if(mModelSolver != nullptr){
            return mModelSolver->checkIk(_pose, _angles, _type);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return false;
        }
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _pose, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
        if(mModelSolver != nullptr){
            return mModelSolver->checkIk(_pose, _angles, _type);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return false;
        }
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f Arm4DoF::pose() const {
        if(mModelSolver != nullptr){
            std::vector<Eigen::Matrix4f> transforms;
            mModelSolver->jointsTransform(transforms);
            return transforms.back();
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return Eigen::Matrix4f::Identity();
        }
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

    //---------------------------------------------------------------------------------------------------------------------
    int Arm4DoF::readPos(int _joint){
        if(mBackend != nullptr){
            return mBackend->jointPos(_joint);
        }else{
            return 0;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    int Arm4DoF::readLoad(int _joint){
        if(mBackend != nullptr){
            return mBackend->jointLoad(_joint);
        }else{
            return 0;
        }
    }
}
