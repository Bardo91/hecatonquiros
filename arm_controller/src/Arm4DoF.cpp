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
#include <ros/package.h>

namespace hecatonquiros{
    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const Backend::Config &_config) {
        mBackend = Backend::create(_config);
        mArmId = _config.armId;

        std::string modelPath = ros::package::getPath("gazebo_simulation") + "/urdf/arm_description.urdf";   // 666 HOW TO MAKE IT GENERIC WITHOUT ROS.
        mRobotModelLoader = robot_model_loader::RobotModelLoader(modelPath);
        mKinematicModel  = mRobotModelLoader.getModel();
        mKinematicState = robot_state::RobotStatePtr(new robot_state::RobotState(mKinematicModel));
        mJointsGroup = mKinematicModel->getJointModelGroup("");
        mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::home(){
        joints({mHome1, mHome2, mHome3, mHome4, mHome5});
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::joints(std::vector<double> _q) {
        assert(_q.size() == mArmJoints.size());

        for(unsigned i = 0; i < _q.size(); i++){
            mArmJoints[i] = _q[i];
        }
        mKinematicState->setJointGroupPositions(mJointsGroup, mArmJoints);
        if(mBackend != nullptr){
            mBackend->joints(std::vector<float>(mArmJoints.begin(), mArmJoints.end()));
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<double> Arm4DoF::joints() {
        mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
        return mArmJoints;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::position(Eigen::Matrix4f _position, float _wirst) {
        joints(mArmJoints); // send joints to arduino
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f Arm4DoF::position() {
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position){
        std::vector<Eigen::Matrix4f> ts;
        std::vector<double> angles;
        return checkIk(_position, angles, ts);
    }


    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position, std::vector<Eigen::Matrix4f> &_transformations){
        std::vector<double> angles;
        return checkIk(_position, angles,_transformations);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position, std::vector<double> &_angles){
        std::vector<Eigen::Matrix4f> ts;
        return checkIk(_position,_angles, ts);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position, std::vector<double> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        Eigen::Affine3f affine3f(_position);
        bool foundIk = mKinematicState->setFromIK(mJointsGroup, affine3f.cast<double>() , 10, 0.1);
        if(foundIk){
            mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
            _angles = mArmJoints;
            return true;
        }else{
            return false;
        }

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::directKinematic(const std::vector<double> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        return false;
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