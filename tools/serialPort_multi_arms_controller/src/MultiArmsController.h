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

#include "IndividualBackend.h"

#include "rapidjson/document.h"

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
    bool clawService1(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool jointsService1(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool jointIDService1(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool loadIDService1(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);

    bool clawService2(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool jointsService2(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool jointIDService2(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);
    bool loadIDService2(hecatonquiros::ReqData::Request  &_req, hecatonquiros::ReqData::Response &_res);

private:
    // std::thread mLoopThread;
    // bool mRun;

    ros::Publisher mStatePublisher;
    ros::Publisher mMovingKeepAlive;

    ros::Publisher mJointsArm1Publisher, mJointsArm2Publisher, mJointIDArm1Publisher, mJointIDArm2Publisher, mLoadIDArm1Publisher, mLoadIDArm2Publisher; 

    ros::ServiceServer mClawArm1Service, mClawArm2Service, mJointsArm1Service, mJointsArm2Service, mJointIDArm1Service, mJointIDArm2Service, mLoadIDArm1Service, mLoadIDArm2Service;

    ros::Subscriber mJointsArm1Subscriber, mJointsArm2Subscriber;

    IndividualBackend mArmsBackend1, mArmsBackend2;
    rapidjson::Document mConfigFile1, mConfigFile2;

    std::vector<IndividualBackend> *mArmsBackend;
    std::vector<rapidjson::Document> *mConfigFiles;
    bool mActuateBackend = false;

    bool mEnableBackend1, mEnableBackend2;
};
