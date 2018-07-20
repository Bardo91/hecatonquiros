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

#include "DualManipulator.h"

#include <iostream>

bool DualManipulator::init(std::string _json){
    hecatonquiros::Backend::Config bc1; 
    hecatonquiros::Backend::Config bc2; 

    if(mConfigFile.Parse(_json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    bool enableBackend = mConfigFile["enable_backend"].GetBool();
    bool visualize = mConfigFile["visualize"].GetBool();

    if(enableBackend){ 
		std::string serialPort = mConfigFile["serial_port"].GetString();
        bc1.configXML = mConfigFile["configXML1"].GetString(); 
		bc2.configXML = mConfigFile["configXML2"].GetString();
		bc1.type = hecatonquiros::Backend::Config::eType::Feetech; bc1.port = serialPort; bc1.armId =1;
		bc2.type = hecatonquiros::Backend::Config::eType::Feetech; bc2.port = serialPort; bc2.armId =2;
    }else{
        bc1.type = hecatonquiros::Backend::Config::eType::Dummy;
        bc2.type = hecatonquiros::Backend::Config::eType::Dummy;
    }

    //arm controller
    hecatonquiros::ModelSolver::Config ms1;
    ms1.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    ms1.robotName = "left_arm";
    ms1.manipulatorName = "manipulator";
    ms1.robotFile = mConfigFile["arm_model"].GetString();
    std::vector<float> off1;
    const rapidjson::Value& o1 = mConfigFile["offset1"];
    for (auto& v : o1.GetArray()){
        off1.push_back(v.GetDouble());
    }
    ms1.offset = off1;
    ms1.rotation = {0,0,1,0};
    ms1.visualizer = visualize;
    mLeftArm = new hecatonquiros::Arm4DoF(ms1, bc1);

    auto ms2 = ms1;
    ms2.robotName = "right_arm";
    std::vector<float> off2;
    const rapidjson::Value& o2 = mConfigFile["offset2"];
    for (auto& v : o2.GetArray()){
        off2.push_back(v.GetDouble());
    }
    ms2.offset = off2;
    ms2.rotation = {0,0,1,0};
    mRightArm = new hecatonquiros::Arm4DoF(ms2, bc2);

    return true;
}
void DualManipulator::joints(eArm _arm, std::vector<float> &_joints, bool _actuateBackend){
    if(_arm == eArm::LEFT){
        mLeftArm->joints(_joints,_actuateBackend);
    }else if(_arm == eArm::RIGHT){
        mRightArm->joints(_joints,_actuateBackend);
    }
}

Eigen::Matrix4f DualManipulator::pose(eArm _arm) {
    if(_arm == eArm::LEFT){
        return mLeftArm->pose();
    }else if(_arm == eArm::RIGHT){
        return mRightArm->pose();
    }else{
        assert(false);
        return Eigen::Matrix4f::Identity();
    }
}


std::vector<float> DualManipulator::joints(eArm _arm) const{
    if(_arm == eArm::LEFT){
        return mLeftArm->joints();
    }else if(_arm == eArm::RIGHT){
        return mRightArm->joints();
    }else{
        return std::vector<float>();
    }
}

bool DualManipulator::checkIk(DualManipulator::eArm _arm, Eigen::Matrix4f _pose, std::vector<float> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
    if(_arm == eArm::LEFT){
        return mLeftArm->checkIk(_pose, _angles, _type);
    }else if(_arm == eArm::RIGHT){
        return mRightArm->checkIk(_pose, _angles, _type);
    }else{
        return false;
    }
}

bool DualManipulator::checkIk(DualManipulator::eArm _arm, Eigen::Matrix4f _pose, std::vector< std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
    if(_arm == eArm::LEFT){
        return mLeftArm->checkIk(_pose, _angles, _type);
    }else if(_arm == eArm::RIGHT){
        return mRightArm->checkIk(_pose, _angles, _type);
    }else{
        return false;
    }
}

/// Pose in cartesian coordinates (meters).
std::vector<bool> DualManipulator::checkIks(DualManipulator::eArm _arm, std::vector<std::shared_ptr<Eigen::Matrix4f>> _poses, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
    std::vector<bool> hasIk(_poses.size());
    _angles.resize(_poses.size());
    for(unsigned i = 0; i < _poses.size();i++){
        auto posePtr = _poses[i];
        hasIk[i] = checkIk(_arm, *posePtr, _angles[i], _type);
    }
    return hasIk;
}