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

#include <string>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <fstream>
#include <chrono>

#include <hecatonquiros/backends/Backend.h> 

#include "rapidjson/document.h"

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#include "hecatonquiros/ReqData.h"

class MultiArmsController{
public:
    bool init(int _argc, char** _argv);
//     bool start();
//     bool stop();

// private:
//     void loop();

private:
    void jointsCallback(const sensor_msgs::JointStateConstPtr& _msg);
    void continuousJointsPub(int _id, int _njoints);

    bool clawService(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool jointsService(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool jointIDService(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool loadIDService(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool torqueIDService(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);

private:
    // std::thread mLoopThread;
    // bool mRun;

    std::vector<std::thread> mReadJointsThread;

    std::vector<ros::Publisher> mJointsArmPublisher, mJointIDArmPublisher, mLoadIDArmPublisher;
    std::vector<ros::Subscriber> mJointsArmSubscriber;
    std::vector<ros::ServiceServer> mClawArmService, mJointIDArmService, mLoadIDArmService, mTorqueIDArmService;

    std::vector<hecatonquiros::Backend *> mBackend;
    rapidjson::Document mConfigFile;
    std::vector<bool> mEnableBackend;

};
