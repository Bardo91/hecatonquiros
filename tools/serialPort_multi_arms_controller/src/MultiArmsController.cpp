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


#include "MultiArmsController.h"

#include <std_msgs/String.h>
#include <fstream>

#include <hecatonquiros/Arm4DoF.h>
#include <hecatonquiros/model_solvers/ModelSolver.h>

#include <chrono>

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::init(int _argc, char** _argv){

	if (_argc != 3) {
        std::cout << "Bad input arguments, please provide only the path of a json config file with the structure detailed in the documentation" << std::endl;
        return false;
    }

    // Arm 1
    std::ifstream rawFile1(_argv[1]);
    if (!rawFile1.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return false;
    }

    std::stringstream strStream1;
    strStream1 << rawFile1.rdbuf(); //read the file
    std::string json1 = strStream1.str(); //str holds the content of the file

    if(mConfigFile1.Parse(json1.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    if(mArmsBackend1.init(json1)){
        std::cout << "Arm 1 created"<< std::endl;
    }else{
        std::cout  << "Error creating arm 1" << std::endl;
        return false;
    }
    
    //////////////////////////////////////////////////
    // Arm 2
    std::ifstream rawFile2(_argv[2]);
    if (!rawFile2.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return false;
    }

    std::stringstream strStream2;
    strStream2 << rawFile2.rdbuf(); //read the file
    std::string json2 = strStream2.str(); //str holds the content of the file

    if(mConfigFile2.Parse(json2.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    if(mArmsBackend2.init(json2)){
        std::cout << "Arm 2 created "<< std::endl;
    }else{
        std::cout  << "Error creating arm 2"<< std::endl;
        return false;
    }
    
    mEnableBackend1 = mConfigFile1["enable_backend"].GetBool();
    mEnableBackend2 = mConfigFile2["enable_backend"].GetBool();

    //////////////////////////////////////////////////
    // ROS publishers and subscribers.
    ros::NodeHandle nh;

    mJointsArm1Publisher = nh.advertise<sensor_msgs::JointState>("/backendROS/arm_1/joints_sub", 1);
    mJointsArm2Publisher = nh.advertise<sensor_msgs::JointState>("/backendROS/arm_2/joints_sub", 1);

    mJointIDArm1Publisher = nh.advertise<std_msgs::Int32>("/backendROS/arm_1/jointId_sub", 1);
    mJointIDArm2Publisher = nh.advertise<std_msgs::Int32>("/backendROS/arm_2/jointId_sub", 1);

    mLoadIDArm1Publisher = nh.advertise<std_msgs::Int32>("/backendROS/arm_1/loadId_sub", 1);
    mLoadIDArm2Publisher = nh.advertise<std_msgs::Int32>("/backendROS/arm_2/loadId_sub", 1);

    std::cout << "Created ROS Publishers" << std::endl;

    //////////////////////////////////////////////////
    mJointsArm1Subscriber = nh.subscribe<sensor_msgs::JointState>("/backendROS/arm_1/joints", 1, \
        [this](const sensor_msgs::JointStateConstPtr& _msg) {
            std::vector<float> jointsArm1;
            jointsArm1.resize(_msg->position.size());
			for(int i = 0; i < _msg->position.size(); i++){
				jointsArm1[i] = _msg->position[i];
			}
            mArmsBackend1.mBackend->joints(jointsArm1, mEnableBackend1);

    	});

    mJointsArm2Subscriber = nh.subscribe<sensor_msgs::JointState>("/backendROS/arm_2/joints", 1, \
        [this](const sensor_msgs::JointStateConstPtr& _msg) {
            std::vector<float> jointsArm2;
            jointsArm2.resize(_msg->position.size());
			for(int i = 0; i < _msg->position.size(); i++){
				jointsArm2[i] = _msg->position[i];
			}

            mArmsBackend2.mBackend->joints(jointsArm2, mEnableBackend2);

    	});

    std::cout << "Created ROS Subscribers" << std::endl;

    //////////////////////////////////////////////////
    mClawArm1Service = nh.advertiseService("/backendROS/arm_1/claw_req", &MultiArmsController::clawService1, this);
    mClawArm2Service = nh.advertiseService("/backendROS/arm_2/claw_req", &MultiArmsController::clawService2, this);

    mJointsArm1Service = nh.advertiseService("/backendROS/arm_1/joints_req", &MultiArmsController::jointsService1, this);
    mJointsArm2Service = nh.advertiseService("/backendROS/arm_2/joints_req", &MultiArmsController::jointsService2, this);

    mJointIDArm1Service = nh.advertiseService("/backendROS/arm_1/jointId_req", &MultiArmsController::jointIDService1, this);
    mJointIDArm2Service = nh.advertiseService("/backendROS/arm_2/jointId_req", &MultiArmsController::jointIDService2, this);

    mLoadIDArm1Service = nh.advertiseService("/backendROS/arm_1/loadId_req", &MultiArmsController::loadIDService1, this);
    mLoadIDArm2Service = nh.advertiseService("/backendROS/arm_2/loadId_req", &MultiArmsController::loadIDService1, this);

    std::cout << "Created ROS Services" << std::endl;

    return true;
}

// //---------------------------------------------------------------------------------------------------------------------
// bool MultiArmsController::start(){

//     mRun = true;
//     mLoopThread = std::thread(&MultiArmsController::loop, this);   

//     return true;
// }

// //---------------------------------------------------------------------------------------------------------------------
// bool MultiArmsController::stop(){
    
//     mRun = false;
//     if(mLoopThread.joinable()) mLoopThread.join();

//     return true;
// }

// //---------------------------------------------------------------------------------------------------------------------
// void MultiArmsController::loop(){
//     while(ros::ok() && mRun){
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
// }

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::clawService1(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    if(_req.req){
        mArmsBackend1.mBackend->claw(_req.id);
    }
    _res.success = true;
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::clawService2(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    if(_req.req){
        mArmsBackend2.mBackend->claw(_req.id);
    }
    _res.success = true;
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::jointsService1(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    std::vector<float> joints;
    if(_req.req){
        joints = mArmsBackend1.mBackend->joints(_req.id);
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    for(auto&j:joints){
        msg.position.push_back(j);
    }
    mJointsArm1Publisher.publish(msg);
    _res.success = true;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::jointsService2(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    std::vector<float> joints;
    if(_req.req){
        joints = mArmsBackend2.mBackend->joints(_req.id);
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    for(auto&j:joints){
        msg.position.push_back(j);
    }
    mJointsArm2Publisher.publish(msg);
    _res.success = true;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::jointIDService1(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    int pos;
    if(_req.req){
        pos = mArmsBackend1.mBackend->jointPos(_req.id);
    }

    std_msgs::Int32 msg;
    msg.data = pos;
    mJointIDArm1Publisher.publish(msg);
    _res.success = true;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::jointIDService2(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    int pos;
    if(_req.req){
        pos = mArmsBackend2.mBackend->jointPos(_req.id);
    }

    std_msgs::Int32 msg;
    msg.data = pos;
    mJointIDArm2Publisher.publish(msg);
    _res.success = true;
 
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::loadIDService1(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    int load;
    if(_req.req){
        load = mArmsBackend1.mBackend->jointLoad(_req.id);
    }

    std_msgs::Int32 msg;
    msg.data = load;
    mLoadIDArm1Publisher.publish(msg);
    _res.success = true;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::loadIDService2(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    int load;
    if(_req.req){
        load = mArmsBackend2.mBackend->jointLoad(_req.id);
    }

    std_msgs::Int32 msg;
    msg.data = load;
    mLoadIDArm2Publisher.publish(msg);
    _res.success = true;

    return true;
}