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

#include <DualManipulator.h>
#include <TopicWatchDog.h>

#include "rapidjson/document.h"

#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

class ManipulatorController{
public:
    bool init(int _argc, char** _argv);
    void start();
    void stop();

private:
    void stateMachine();

private:
    bool leftClawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res);
    bool rightClawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res);
    bool emergencyStopService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res);

    void movingCallback();
    void publisherLoop(DualManipulator::eArm _arm);
private:
    ros::AsyncSpinner *mRosSpinner; // Use 4 threads

    enum class STATES {STOP, HOME, IDLE, MOVING, ERROR};
    STATES mState = STATES::STOP;
    std::string mLastError ="";
    ros::Publisher mStatePublisher;

    std::thread mMainThread;
    std::thread mStatePublisherThread;
    std::thread mLeftJointsPublisherThread, mRightJointsPublisherThread;
    bool mRunning = false;

    ros::Subscriber mLeftTargetJointsSubscriber, mRightTargetJointsSubscriber;
    ros::Subscriber mLeftTargetPose3DSubscriber, mRightTargetPose3DSubscriber;
    ros::Subscriber mLeftTargetPose6DSubscriber, mRightTargetPose6DSubscriber;
    ros::ServiceServer mLeftClawService, mRightClawService;
    ros::ServiceServer mEmergencyStopService;
    bool mEmergencyStop = false;

    TopicWatchDog<geometry_msgs::PoseStamped> *mCrawlerPoseSubscriber;

    DualManipulator mManipulator;
    int mNdof;
    std::vector<float> cHomeJoints;

    std::vector<float> mLeftTargetJoints, mRightTargetJoints;

    rapidjson::Document mConfigFile;
    bool mActuateBackend = false;
    hecatonquiros::ModelSolver *mModelSolverLeft, *mModelSolverRight;
};