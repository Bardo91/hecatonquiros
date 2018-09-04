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
#include <fstream>
#include <chrono>

#include <hecatonquiros/Arm4DoF.h>
#include <hecatonquiros/model_solvers/ModelSolver.h>

#include "TopicWatchDog.h"

#include "rapidjson/document.h"

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>

class IndividualArmController{
public:
    typedef TopicWatchDog<sensor_msgs::JointState> WatchdogJoints;
    typedef TopicWatchDog<geometry_msgs::PoseStamped> WatchdogPose;

    bool init(int _argc, char** _argv);
    void start();
    void stop();

private:
    void stateMachine();

private:
    bool clawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res);
    bool emergencyStopService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res);

    void MovingArmThread();
    void publisherLoop();

    void rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose);
    void eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg);

private:

    void jointsCallback(const sensor_msgs::JointState::ConstPtr &_msg);
    void pose3DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);
    void pose6DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);

    void pose3DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);
    void pose6DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);

private:
    ros::AsyncSpinner *mRosSpinner; // Use 4 threads

    enum class STATES {STOP, HOME, IDLE, MOVING, ERROR, DISABLE};
    STATES mState = STATES::STOP;
    std::string mLastError ="";
    ros::Publisher mStatePublisher;
    ros::Publisher mMovingKeepAlive;

    std::thread mMainThread;
    std::thread mStatePublisherThread;
    std::thread mJointsPublisherThread;
    std::thread mLastAimedJointsThread;
    bool mRunning = false;

    WatchdogJoints *mTargetJointsSubscriber;
    WatchdogPose *mTargetPose3DSubscriber;
    WatchdogPose *mTargetPose6DSubscriber;
    WatchdogPose *mTargetPose3DJacobiSubscriber;
    WatchdogPose *mTargetPose6DJacobiSubscriber;

    ros::ServiceServer mClawService;
    ros::ServiceServer mEmergencyStopService;
    bool mEmergencyStop = false;

    hecatonquiros::Arm4DoF *mArm;
    int mNdof;
    std::vector<float> cHomeJoints;

    std::vector<float> mTargetJoints;
    std::vector<float> mLastAimedJoints;

    rapidjson::Document mConfigFile;
    bool mActuateBackend = false;
    std::string mName = "";
};
