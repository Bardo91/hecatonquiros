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

#include "ManipulatorController.h"

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

#include <hecatonquiros/Arm4DoF.h>
#include <hecatonquiros/model_solvers/ModelSolver.h>

#include <chrono>

//---------------------------------------------------------------------------------------------------------------------
bool ManipulatorController::init(int _argc, char** _argv){
    ros::init(_argc, _argv, "manipulator_controller");
    
    mRosSpinner = new ros::AsyncSpinner(4); // Use 4 threads
    mRosSpinner->start();

	if (_argc != 2) {
        std::cout << "Bad input arguments, please provide only the path of a json config file with the structure detailed in the documentation" << std::endl;
        return false;
    }

    std::ifstream rawFile(_argv[1]);
    if (!rawFile.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return false;
    }

    std::stringstream strStream;
    strStream << rawFile.rdbuf(); //read the file
    std::string json = strStream.str(); //str holds the content of the file

    if(mConfigFile.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }
    
    mActuateBackend = mConfigFile["enable_backend"].GetBool();

    if(mManipulator.init(json)){
        std::cout << "Arm created" << std::endl;
    }else{
        std::cout  << "Error creating arm" <<std::endl;
        return false;
    }
    mNdof = mManipulator.joints(DualManipulator::eArm::LEFT).size();
    // std::cout << "Manipulator has " << mNdof << " dof." << std::endl;

    if(mNdof == 3)
        cHomeJoints = {0,-M_PI/5,M_PI/1.5};
    if(mNdof == 4)
        cHomeJoints = {0,-M_PI/5,M_PI/1.5, -90*M_PI/180.0};
    if(mNdof == 6)
        cHomeJoints = {0,-M_PI/6,M_PI/1.5, -90*M_PI/180.0, /*-M_PI/1.5*/ 0,0};

    // ROS publishers and subscribers.
    ros::NodeHandle nh;
    
    mStatePublisher = nh.advertise<std_msgs::String>("/manipulator_controller/state", 1);  
    
    mLeftTargetJointsSubscriber = nh.subscribe<sensor_msgs::JointState>("/hecatonquiros/left/target_joints", 1, [&](const sensor_msgs::JointState::ConstPtr &_msg){
        if(mState != STATES::STOP && mState !=STATES::IDLE && mState != STATES::HOME){
            std::vector<float> joints;
            for(auto j:_msg->position){
                joints.push_back(j);
            }
            mLeftTargetJoints = joints; // 666 Thread safe?
        }
    });

    mRightTargetJointsSubscriber = nh.subscribe<sensor_msgs::JointState>("/hecatonquiros/right/target_joints", 1, [&](const sensor_msgs::JointState::ConstPtr &_msg){
        if(mState != STATES::STOP && mState !=STATES::IDLE && mState != STATES::HOME){
            std::vector<float> joints;
            for(auto j:_msg->position){
                joints.push_back(j);
            }
            mRightTargetJoints = joints; // 666 Thread safe?
        }
    });

    mLeftTargetPose3DSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/left/target_pose3d", 1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState != STATES::STOP && mState !=STATES::IDLE && mState != STATES::HOME){

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose(0,3) = _msg->pose.position.x;
            pose(1,3) = _msg->pose.position.y;
            pose(2,3) = _msg->pose.position.z;
            
            std::vector<float> joints;
            if(mManipulator.checkIk(DualManipulator::eArm::LEFT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
                mLeftTargetJoints = joints; // 666 Thread safe?
            }else{
		std::cout << "Failed IK" << std::endl;
	    }
        }
    });

    mRightTargetPose3DSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/right/target_pose3d", 1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState != STATES::STOP && mState !=STATES::IDLE && mState != STATES::HOME){

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose(0,3) = _msg->pose.position.x;
            pose(1,3) = _msg->pose.position.y;
            pose(2,3) = _msg->pose.position.z;
            
            std::vector<float> joints;
            if(mManipulator.checkIk(DualManipulator::eArm::RIGHT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
                mRightTargetJoints = joints; // 666 Thread safe?
            }else{
		std::cout << "Failed IK" << std::endl;
	    }
        }
    });

    mLeftTargetPose6DSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/left/target_pose6d", 1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState != STATES::STOP && mState !=STATES::IDLE && mState != STATES::HOME){
            
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose(0,3) = _msg->pose.position.x;
            pose(1,3) = _msg->pose.position.y;
            pose(2,3) = _msg->pose.position.z;

            float dx = _msg->pose.orientation.x;
            float dy = _msg->pose.orientation.y;
            float dz = _msg->pose.orientation.z;

            Eigen::Vector3f zAxis = {dx, dy, dz};
            zAxis /=zAxis.norm();
            // zAxis -= zAxis.dot(xAxis)*xAxis;

            // Eigen::Vector3f yAxis = zAxis.cross(xAxis);
            // pose.block<3,1>(0,0) = xAxis;
            // pose.block<3,1>(0,1) = yAxis;
            pose.block<3,1>(0,2) = zAxis;
            
            std::vector<float> joints;
            if(mManipulator.checkIk(DualManipulator::eArm::LEFT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
                mLeftTargetJoints = joints; // 666 Thread safe?
            }  
        }
    });

    mRightTargetPose6DSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/right/target_pose6d", 1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState != STATES::STOP && mState !=STATES::IDLE && mState != STATES::HOME){

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose(0,3) = _msg->pose.position.x;
            pose(1,3) = _msg->pose.position.y;
            pose(2,3) = _msg->pose.position.z;

            float dx = _msg->pose.orientation.x;
            float dy = _msg->pose.orientation.y;
            float dz = _msg->pose.orientation.z;

            Eigen::Vector3f zAxis = {dx, dy, dz};
            zAxis /=zAxis.norm();
            // zAxis -= zAxis.dot(xAxis)*xAxis;

            // Eigen::Vector3f yAxis = zAxis.cross(xAxis);
            // pose.block<3,1>(0,0) = xAxis;
            // pose.block<3,1>(0,1) = yAxis;
            pose.block<3,1>(0,2) = zAxis;
            
            std::vector<float> joints;
            if(mManipulator.checkIk(DualManipulator::eArm::RIGHT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
                mRightTargetJoints = joints; // 666 Thread safe?
            }
        }
    });

    mLeftClawService = nh.advertiseService("/hecatonquiros/left/claw", &ManipulatorController::leftClawService, this);
    mRightClawService = nh.advertiseService("/hecatonquiros/right/claw", &ManipulatorController::rightClawService, this);
    mEmergencyStopService = nh.advertiseService("/hecatonquiros/emergency_stop", &ManipulatorController::emergencyStopService, this);


    return true;
}


//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::start(){
    mRunning = true;
    mState = STATES::HOME;

    mStatePublisherThread = std::thread([&](){
        while(mRunning && ros::ok()){
            std_msgs::String msgState;
            switch(mState){
                case STATES::STOP:
                    msgState.data = "STOP";
                    break;
                case STATES::HOME:
                    msgState.data = "HOME";
                    break;
                case STATES::IDLE:
                    msgState.data = "IDLE";
                    break;
                case STATES::MOVING:
                    msgState.data = "MOVING";
                    break;
                case STATES::ERROR:
                    msgState.data = "ERROR - "+mLastError;
                    break;
            }
            mStatePublisher.publish(msgState);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    mMainThread = std::thread(&ManipulatorController::stateMachine, this);

    mLeftJointsPublisherThread = std::thread(&ManipulatorController::publisherLoop, this, DualManipulator::eArm::LEFT);
    mRightJointsPublisherThread = std::thread(&ManipulatorController::publisherLoop, this, DualManipulator::eArm::RIGHT);

}

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::stop(){
    mRunning = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mLastError = "Intentionally stoped";
    mState = STATES::ERROR;
    if(mStatePublisherThread.joinable())    mStatePublisherThread.join();
    if(mMainThread.joinable())              mMainThread.join();

}


//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::stateMachine(){
    while(ros::ok() && mRunning){
        switch(mState){
            case STATES::STOP:
                mManipulator.mLeftArm->openClaw();
                mManipulator.mRightArm->openClaw();
                // 666 disable motors
                break;
            case STATES::HOME:
                mManipulator.mLeftArm->joints(cHomeJoints, mActuateBackend);
                mManipulator.mRightArm->joints(cHomeJoints, mActuateBackend);
                mManipulator.mLeftArm->openClaw();
                mManipulator.mRightArm->openClaw();
                mState = STATES::MOVING;
                break;
            case STATES::IDLE:
                mManipulator.mLeftArm->joints(cHomeJoints, mActuateBackend);
                mManipulator.mRightArm->joints(cHomeJoints, mActuateBackend);
                mManipulator.mLeftArm->openClaw();
                mManipulator.mRightArm->openClaw();
                break;
            case STATES::MOVING:
                movingCallback();
                break;
            case STATES::ERROR:
            
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}


bool ManipulatorController::leftClawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res){
    if(_req.data)
        mManipulator.mLeftArm->openClaw();
    else
        mManipulator.mLeftArm->closeClaw();

    _res.success = true;
    return true;
};

bool ManipulatorController::rightClawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res){
    if(_req.data)
        mManipulator.mRightArm->openClaw();
    else
        mManipulator.mRightArm->closeClaw();

    _res.success = true;
    return true;
};


bool ManipulatorController::emergencyStopService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res){
    mEmergencyStop = _req.data;

    if(mEmergencyStop){
        mState = STATES::STOP;
        mManipulator.mLeftArm->openClaw();
        mManipulator.mRightArm->openClaw();
    }

    _res.success = true;
    return true;
}

void ManipulatorController::movingCallback(){
    auto moveIncLambda = [&](std::vector<float> _targetJoints, DualManipulator::eArm _arm){  // INTEGRATE IN callback to limit speed
        std::vector<float> currJoints = mManipulator.joints(_arm); // 666 thread safe?
        float MAX_JOINT_DIST = 5.0*M_PI/180.0; // 666 parametrize
        for(unsigned i = 0; i < _targetJoints.size();i++){
            float distJoint = (_targetJoints[i] - currJoints[i]);                  // 666 MOVE TO MANIPULATOR CONTROLLER
            distJoint = distJoint > MAX_JOINT_DIST ? MAX_JOINT_DIST:distJoint;
            _targetJoints[i] = currJoints[i] + distJoint;
        }
        mManipulator.joints(_arm,_targetJoints, mActuateBackend);
    };

    moveIncLambda(mLeftTargetJoints, DualManipulator::eArm::LEFT);
    moveIncLambda(mRightTargetJoints, DualManipulator::eArm::RIGHT);
}

void ManipulatorController::publisherLoop(DualManipulator::eArm _arm){
    
    ros::NodeHandle nh;
    std::string armName = _arm == DualManipulator::eArm::LEFT ? "left"  : "right" ;
    ros::Publisher jointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/"+armName+"/joints_state", 1);
    ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/"+armName+"/pose", 1);
    ros::Rate rate(50);

    while(ros::ok()){
        std::vector<float> joints;
        sensor_msgs::JointState jointsMsg;
	jointsMsg.header.stamp = ros::Time::now();
        joints = mManipulator.joints(_arm);
        for(auto&j:joints){
            jointsMsg.position.push_back(j);
        }
        jointsPublisher.publish(jointsMsg);
	
	Eigen::Matrix4f pose = mManipulator.pose(_arm);

	geometry_msgs::PoseStamped poseMsg;
	poseMsg.pose.position.x = pose(0,3);
	poseMsg.pose.position.y = pose(1,3);
	poseMsg.pose.position.z = pose(2,3);
	Eigen::Quaternionf q = Eigen::Quaternionf(pose.block<3,3>(0,0).matrix());
	poseMsg.pose.orientation.w = q.w();
	poseMsg.pose.orientation.x = q.x();
	poseMsg.pose.orientation.y = q.y();
	poseMsg.pose.orientation.z = q.z();
	poseMsg.header.frame_id = "hecatonquiros_dual";
	poseMsg.header.stamp = poseMsg.header.stamp ;
	posePublisher.publish(poseMsg);
        rate.sleep();
    }
}   
