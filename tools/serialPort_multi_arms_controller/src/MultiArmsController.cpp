//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Manuel Perez Jimenez (a.k.a. manuoso) manuperezj@gmail.com
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

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::init(int _argc, char** _argv){

    ros::NodeHandle nh;
    mConfigService = nh.advertiseService("/backendROS/config_req", &MultiArmsController::configService, this);
    std::cout << "Created ROS Config Service" << std::endl;

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
bool MultiArmsController::configService(hecatonquiros::ConfigData::Request &_req, hecatonquiros::ConfigData::Response &_res){
    
    ros::NodeHandle nh;
    hecatonquiros::Backend::Config bc; 

    std::cout << "ID: " << _req.id << std::endl;
    std::cout << "NDOF: " << _req.ndof << std::endl;
    std::cout << "ConfigXML: " << _req.configxml << std::endl;
    std::cout << "SerialPort: " << _req.serialport << std::endl;

    bc.configXML = _req.configxml; 
    bc.type = hecatonquiros::Backend::Config::eType::Feetech; 
    bc.port = _req.serialport;
    bc.armId = _req.id;

    mBackend.push_back(hecatonquiros::Backend::create(bc));
    std::cout << "Arm " << std::to_string(_req.id) << " created"<< std::endl;

    //////////////////////////////////////////////////
    // ROS publishers
    mMovingKeepAlive.push_back(nh.advertise<std_msgs::String>("/backendFeetech/arm_"+std::to_string(_req.id)+"/moving/keep_alive", 1));  
    mJointsArmPublisher.push_back(nh.advertise<sensor_msgs::JointState>("/backendROS/arm_"+std::to_string(_req.id)+"/joints_sub", 1));
    std::cout << "Created ROS Publishers of Arm " << std::to_string(_req.id) << std::endl;

    //////////////////////////////////////////////////
    // ROS subscribers
    mJointsArmSubscriber.push_back(nh.subscribe<sensor_msgs::JointState>("/backendROS/arm_"+std::to_string(_req.id)+"/joints", 1, &MultiArmsController::jointsCallback, this));
    std::cout << "Created ROS Subscribers of Arm " << std::to_string(_req.id) << std::endl;

    //////////////////////////////////////////////////
    // ROS services
    mClawArmService.push_back(nh.advertiseService("/backendROS/arm_"+std::to_string(_req.id)+"/claw_req", &MultiArmsController::clawService, this));
    std::cout << "Created ROS Services of Arm " << std::to_string(_req.id) << std::endl;

    mReadJointsThread.push_back(std::thread(&MultiArmsController::continuousJointsPub, this, _req.id, _req.ndof));
    std::cout << "Created thread Continuous Joints publisher of Arm " << std::to_string(_req.id) << std::endl;

    _res.success = true;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MultiArmsController::clawService(hecatonquiros::ReqData::Request &_req, hecatonquiros::ReqData::Response &_res){
    
    if(_req.req){
        mBackend[_req.id-1]->claw(_req.data);
    }
    _res.success = true;
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void MultiArmsController::jointsCallback(const sensor_msgs::JointStateConstPtr& _msg){

    std::vector<float> jointsArm;
    for(int j = 0; j < _msg->position.size(); j++){
        jointsArm.push_back(_msg->position[j]);
    }
    int id_arm = _msg->effort[0];
    mBackend[id_arm-1]->joints(jointsArm, true);

    std_msgs::String aliveMsg;  
    aliveMsg.data = "I am moving "+id_arm; 
    mMovingKeepAlive[id_arm-1].publish(aliveMsg);

}

//---------------------------------------------------------------------------------------------------------------------
void MultiArmsController::continuousJointsPub(int _id, int _njoints){
    
    //ros::Rate rate(10);

    while(ros::ok()){

        std::vector<float> joints = mBackend[_id-1]->joints(_njoints);

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        for(auto&j:joints){
            msg.position.push_back(j);
        }
        mJointsArmPublisher[_id-1].publish(msg);

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //rate.sleep();

    }

}
