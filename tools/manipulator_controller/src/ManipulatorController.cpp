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
    
    mStatePublisher = nh.advertise<std_msgs::String>("/hecatonquiros/controller/state", 1);  
    mMovingKeepAlive = nh.advertise<std_msgs::String>("/hecatonquiros/moving/keep_alive", 1);  

    // Direct joints access
    mLeftTargetJointsSubscriber = new WatchdogJoints("/hecatonquiros/left/target_joints", 0.2);
    WatchdogJoints::Callback wrapperLeftJointsCallback = [&](const typename sensor_msgs::JointState::ConstPtr &_msg){rightJointsCallback(_msg);};
    mLeftTargetJointsSubscriber->attachCallback(wrapperLeftJointsCallback);

    mRightTargetJointsSubscriber = new WatchdogJoints("/hecatonquiros/right/target_joints", 0.2);
    WatchdogJoints::Callback wrapperRightJointsCallback = [&](const typename sensor_msgs::JointState::ConstPtr &_msg){leftJointsCallback(_msg);};
    mRightTargetJointsSubscriber->attachCallback(wrapperRightJointsCallback);

    // FAST-IK methods
    mLeftTargetPose3DSubscriber = new WatchdogPose("/hecatonquiros/left/target_pose3d", 0.2);
    WatchdogPose::Callback wrapperLeftPose3dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){leftPose3DCallback(_msg);};
    mLeftTargetPose3DSubscriber->attachCallback(wrapperLeftPose3dCallback);

    mRightTargetPose3DSubscriber = new WatchdogPose("/hecatonquiros/right/target_pose3d", 0.2);
    WatchdogPose::Callback wrapperRightPose3dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){rightPose3DCallback(_msg);};
    mRightTargetPose3DSubscriber->attachCallback(wrapperRightPose3dCallback);

    mLeftTargetPose6DSubscriber = new WatchdogPose("/hecatonquiros/left/target_pose6d", 0.2);
    WatchdogPose::Callback wrapperLeftPose6dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){leftPose6DCallback(_msg);};
    mLeftTargetPose6DSubscriber->attachCallback(wrapperLeftPose6dCallback);

    mRightTargetPose6DSubscriber = new WatchdogPose("/hecatonquiros/right/target_pose6d", 0.2);
    WatchdogPose::Callback wrapperRightPose6dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){rightPose6DCallback(_msg);};
    mRightTargetPose6DSubscriber->attachCallback(wrapperRightPose6dCallback);

    // Jacobian methods
    mLeftTargetPose3DJacobiSubscriber = new WatchdogPose("/hecatonquiros/left/target_pose3d_jacobi", 0.2);
    WatchdogPose::Callback wrapperLeftPose3dJacobiCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){leftPose3DJacobiCallback(_msg);};
    mLeftTargetPose3DJacobiSubscriber->attachCallback(wrapperLeftPose3dJacobiCallback);

    mRightTargetPose3DJacobiSubscriber = new WatchdogPose("/hecatonquiros/right/target_pose3d_jacobi", 0.2);
    WatchdogPose::Callback wrapperRightPose3dJacobiCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){rightPose3DJacobiCallback(_msg);};
    mRightTargetPose3DJacobiSubscriber->attachCallback(wrapperRightPose3dJacobiCallback);

    mLeftTargetPose6DJacobiSubscriber = new WatchdogPose("/hecatonquiros/left/target_pose6d_jacobi", 0.2);
    WatchdogPose::Callback wrapperLeftPose6dJacobiCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){leftPose6DJacobiCallback(_msg);};
    mLeftTargetPose6DJacobiSubscriber->attachCallback(wrapperLeftPose6dJacobiCallback);

    mRightTargetPose6DJacobiSubscriber = new WatchdogPose("/hecatonquiros/right/target_pose6d_jacobi", 0.2);
    WatchdogPose::Callback wrapperRightPose6dJacobiCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){rightPose6DJacobiCallback(_msg);};
    mRightTargetPose6DJacobiSubscriber->attachCallback(wrapperRightPose6dJacobiCallback);

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
                mLeftTargetJoints = cHomeJoints;
                mRightTargetJoints = cHomeJoints;
                mManipulator.mLeftArm->joints(cHomeJoints, mActuateBackend);
                mManipulator.mRightArm->joints(cHomeJoints, mActuateBackend);
                break;
            case STATES::IDLE:
                mLeftTargetJoints = cHomeJoints;
                mRightTargetJoints = cHomeJoints;
                mManipulator.mLeftArm->joints(cHomeJoints, mActuateBackend);
                mManipulator.mRightArm->joints(cHomeJoints, mActuateBackend);
                break;
            case STATES::MOVING:
                movingCallback();
                break;
            case STATES::ERROR:
                break;
        }

	if( mLeftTargetJointsSubscriber->isValid() ||
	    mRightTargetJointsSubscriber->isValid() ||
	    mLeftTargetPose3DSubscriber->isValid() || 
	    mRightTargetPose3DSubscriber->isValid() ||
	    mLeftTargetPose6DSubscriber->isValid() ||
	    mRightTargetPose6DSubscriber->isValid() ||
	    mLeftTargetPose3DJacobiSubscriber->isValid() || 
	    mRightTargetPose3DJacobiSubscriber->isValid() ||
	    mLeftTargetPose6DJacobiSubscriber->isValid() ||
	    mRightTargetPose6DJacobiSubscriber->isValid() )
	{
		mState = STATES::MOVING;
	}else{		
        if(mState != STATES::HOME){
            mManipulator.mLeftArm->openClaw();
            mManipulator.mRightArm->openClaw();
		    mState = STATES::HOME;
        }
	}	

        //std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

//---------------------------------------------------------------------------------------------------------------------

bool ManipulatorController::leftClawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res){
    if(_req.data)
        mManipulator.mLeftArm->openClaw();
    else
        mManipulator.mLeftArm->closeClaw();

    _res.success = true;
    return true;
};

//---------------------------------------------------------------------------------------------------------------------
bool ManipulatorController::rightClawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res){
    if(_req.data)
        mManipulator.mRightArm->openClaw();
    else
        mManipulator.mRightArm->closeClaw();

    _res.success = true;
    return true;
};


//---------------------------------------------------------------------------------------------------------------------
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

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::movingCallback(){
    auto moveIncLambda = [&](std::vector<float> _targetJoints, DualManipulator::eArm _arm)->std::vector<float>{  // INTEGRATE IN callback to limit speed
        std::vector<float> currJoints = mManipulator.joints(_arm); // 666 thread safe?
        float MAX_JOINT_DIST = 10.0*M_PI/180.0; // 666 parametrize
        for(unsigned i = 0; i < _targetJoints.size();i++){
            float distJoint = (_targetJoints[i] - currJoints[i]);                  // 666 MOVE TO MANIPULATOR CONTROLLER
            distJoint = distJoint > MAX_JOINT_DIST ? MAX_JOINT_DIST:distJoint;
            _targetJoints[i] = currJoints[i] + distJoint;
        }
        mManipulator.joints(_arm,_targetJoints, mActuateBackend);
        return _targetJoints;
    };

    mLeftLastAimedJoints = moveIncLambda(mLeftTargetJoints, DualManipulator::eArm::LEFT);
    mRightLastAimedJoints = moveIncLambda(mRightTargetJoints, DualManipulator::eArm::RIGHT);
    
    
    std_msgs::String aliveMsg;  aliveMsg.data = "I am moving"; 
    mMovingKeepAlive.publish(aliveMsg);
}

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::publisherLoop(DualManipulator::eArm _arm){
    ros::NodeHandle nh;
    std::string armName = _arm == DualManipulator::eArm::LEFT ? "left"  : "right" ;
    bool isLeft = (_arm == DualManipulator::eArm::LEFT);
    ros::Publisher jointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/"+armName+"/joints_state", 1);
    ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/"+armName+"/pose", 1);
    ros::Publisher aimingJointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/"+armName+"/aiming_joints", 1);  

    ros::Rate rate(50);

    while(ros::ok()){
        // Publish state joints
        std::vector<float> joints;
        sensor_msgs::JointState jointsMsg;
	    jointsMsg.header.stamp = ros::Time::now();
        joints = mManipulator.joints(_arm);
        for(auto&j:joints){
            jointsMsg.position.push_back(j);
        }
        jointsPublisher.publish(jointsMsg);

        // Publish aiming joints
        sensor_msgs::JointState aimingJointsMsg;
	    aimingJointsMsg.header.stamp = jointsMsg.header.stamp;
        std::vector<float> aimJoints = isLeft?mLeftLastAimedJoints: mRightLastAimedJoints;
        for(auto&j:aimJoints){
            aimingJointsMsg.position.push_back(j);
        }
        aimingJointsPublisher.publish(aimingJointsMsg);

        // Publish current pose
        Eigen::Matrix4f pose = mManipulator.pose(_arm);
        geometry_msgs::PoseStamped poseMsg;
        eigenToRos(pose, poseMsg);
        poseMsg.header.frame_id = "hecatonquiros_dual";
        poseMsg.header.stamp = jointsMsg.header.stamp;
        posePublisher.publish(poseMsg);
        rate.sleep();
    }
}   

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose){
	_pose = Eigen::Matrix4f::Identity();
	_pose(0,3) = _msg->pose.position.x;
	_pose(1,3) = _msg->pose.position.y;
	_pose(2,3) = _msg->pose.position.z;

	Eigen::Quaternionf q;
	q.x() = _msg->pose.orientation.x;
	q.y() = _msg->pose.orientation.y;
	q.z() = _msg->pose.orientation.z;
	q.w() = _msg->pose.orientation.w;

	_pose.block<3,3>(0,0) = q.matrix();
}

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg){
	_msg.pose.position.x = _pose(0,3);
	_msg.pose.position.y = _pose(1,3);
	_msg.pose.position.z = _pose(2,3);

	Eigen::Quaternionf q(_pose.block<3,3>(0,0));
	_msg.pose.orientation.x = q.x();
	_msg.pose.orientation.y = q.y();
	_msg.pose.orientation.z = q.z();
	_msg.pose.orientation.w = q.w();

}

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::rightJointsCallback(const sensor_msgs::JointState::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            std::vector<float> joints;
            for(auto j:_msg->position){
                joints.push_back(j);
            }
            mLeftTargetJoints = joints; // 666 Thread safe?
        }
    };

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::leftJointsCallback(const sensor_msgs::JointState::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            std::vector<float> joints;
            for(auto j:_msg->position){
                joints.push_back(j);
            }
            mRightTargetJoints = joints; // 666 Thread safe?
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::rightPose3DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
	if(mState == STATES::MOVING){
	    Eigen::Matrix4f pose;
	    rosToEigen(_msg, pose);
	    
	    std::vector<float> joints;
	    if(mManipulator.checkIk(DualManipulator::eArm::RIGHT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
		mRightTargetJoints = joints; // 666 Thread safe?
	    }else{
		    std::cout << "Failed IK right" << std::endl;
	    }
	}
}

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::leftPose3DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){	    
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);

            std::vector<float> joints;
            if(mManipulator.checkIk(DualManipulator::eArm::LEFT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
                mLeftTargetJoints = joints; // 666 Thread safe?
            }else{
                std::cout << "Failed IK left" << std::endl;
            }
        }
    }

void ManipulatorController::rightPose6DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);

            std::vector<float> joints;
            if(mManipulator.checkIk(DualManipulator::eArm::RIGHT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
                mRightTargetJoints = joints; // 666 Thread safe?
            }else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::leftPose6DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);
                
            std::vector<float> joints;
            if(mManipulator.checkIk(DualManipulator::eArm::LEFT, pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
                mLeftTargetJoints = joints; // 666 Thread safe?
            }  else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }


//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::rightPose3DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
	if(mState == STATES::MOVING){
	    Eigen::Matrix4f pose;
	    rosToEigen(_msg, pose);

	    std::vector<float> joints;
	    Eigen::Vector3f position = pose.block<3,1>(0,3);
	    if(mManipulator.mRightArm->jacobianStep(position, joints)){
		    mRightTargetJoints = joints; // 666 Thread safe?
	    }else{
		    std::cout << "Failed IK right" << std::endl;
	    }
	}
}

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::leftPose3DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);

            std::vector<float> joints;
            Eigen::Vector3f position = pose.block<3,1>(0,3);
            if(mManipulator.mLeftArm->jacobianStep(position, joints)){
                mLeftTargetJoints = joints; // 666 Thread safe?
            }else{
                std::cout << "Failed IK left" << std::endl;
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::rightPose6DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);
            
            std::vector<float> joints;
            if(mManipulator.mLeftArm->jacobianStep(pose, joints)){
                mRightTargetJoints = joints; // 666 Thread safe?
            }else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void ManipulatorController::leftPose6DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);
                
            std::vector<float> joints;
            if(mManipulator.mRightArm->jacobianStep(pose, joints)){
                mLeftTargetJoints = joints; // 666 Thread safe?
            }  else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }



