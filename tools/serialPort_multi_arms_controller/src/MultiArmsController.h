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

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "hecatonquiros/ReqData.h"
#include "hecatonquiros/ConfigData.h"

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
    bool configService(hecatonquiros::ConfigData::Request  &_req, hecatonquiros::ConfigData::Response &_res);

private:
    // std::thread mLoopThread;
    // bool mRun;

    std::map<int, std::thread *> mReadJointsThread;

    std::map<int, ros::Publisher> mJointsArmPublisher, mMovingKeepAlive;
    std::map<int, ros::Subscriber> mJointsArmSubscriber;
    std::map<int, ros::ServiceServer> mClawArmService;
    ros::ServiceServer mConfigService;

    std::map<int,hecatonquiros::Backend *> mBackend;

};
