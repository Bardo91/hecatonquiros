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

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <fstream>
#include <chrono>

#include "rapidjson/document.h"

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

class ArmJoystick{
public:
    bool init(int _argc, char** _argv);
    bool start();
    bool stop();

private:
    void loop();

private:
    void joystickThread();
    void movingFunction();
    void stopFunction();

private:
    void rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose);
    void eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg);

private:
    enum class STATES {STOP, MOVING, ERROR};
    STATES mState = STATES::STOP;

    rapidjson::Document mConfigFile;

    std::thread mLoopThread, mJoyThread;
    bool mRun;

    ros::Subscriber mPoseLeftArmSubscriber, mPoseRightArmSubscriber, mJoySubscriber;
    ros::Publisher mTargetLeftArmPublisher, mTargetRightArmPublisher;

    Eigen::Matrix4f mPoseLeft, mPoseRight, mCurrentPoseLeft, mCurrentPoseRight;

    std::string mLeftArmPub, mRightArmPub, mLeftArmSub, mRightArmSub, mJoySub;

    bool mSecButLeft, mSecButRight;

    double mVel = 0.0;
    double mVelX1, mVelX2, mVelY1, mVelY2, mVelZ1, mVelZ2Up, mVelZ2Down;
    std::vector<float> mJoyAxes;
    std::vector<int>  mJoyButtons;

};
