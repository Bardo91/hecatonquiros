//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#include "IndividualArm.h"

#include <iostream>

bool IndividualArm::init(std::string _json){
    hecatonquiros::Backend::Config bc; 

    if(mConfigFile.Parse(_json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    std::string name = mConfigFile["name"].GetString();
    bool enableBackend = mConfigFile["enable_backend"].GetBool();
    bool visualize = mConfigFile["visualize"].GetBool();

    if(enableBackend){ 
		bc.type = hecatonquiros::Backend::Config::eType::ROS; 
        bc.armId = mConfigFile["id"].GetInt();
    }else{
        bc.type = hecatonquiros::Backend::Config::eType::Dummy;
    }

    //arm controller
    hecatonquiros::ModelSolver::Config ms;
    ms.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    ms.robotName = name;
    ms.manipulatorName = "manipulator";
    ms.robotFile = mConfigFile["arm_model"].GetString();
    std::vector<float> off;
    const rapidjson::Value& o = mConfigFile["offset"];
    for (auto& v : o.GetArray()){
        off.push_back(v.GetDouble());
    }
    ms.offset = off;
    ms.rotation = {0,0,0,1};
    ms.visualizer = visualize;
    mArm = new hecatonquiros::Arm4DoF(ms, bc);

    return true;
}
void IndividualArm::joints(std::vector<float> &_joints, bool _actuateBackend){
    mArm->joints(_joints,_actuateBackend);
}

Eigen::Matrix4f IndividualArm::pose() {
    return mArm->pose();
}


std::vector<float> IndividualArm::joints() const{
    return mArm->joints();
}

bool IndividualArm::checkIk(Eigen::Matrix4f _pose, std::vector<float> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
    return mArm->checkIk(_pose, _angles, _type);
}

bool IndividualArm::checkIk(Eigen::Matrix4f _pose, std::vector< std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
    return mArm->checkIk(_pose, _angles, _type);
}

/// Pose in cartesian coordinates (meters).
std::vector<bool> IndividualArm::checkIks(std::vector<std::shared_ptr<Eigen::Matrix4f>> _poses, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
    std::vector<bool> hasIk(_poses.size());
    _angles.resize(_poses.size());
    for(unsigned i = 0; i < _poses.size();i++){
        auto posePtr = _poses[i];
        hasIk[i] = checkIk(*posePtr, _angles[i], _type);
    }
    return hasIk;
}
