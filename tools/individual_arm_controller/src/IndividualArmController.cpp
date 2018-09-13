//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Manuel Perez Jimenez (a.k.a. manuoso) manuperezj@gmail.com
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


#include "IndividualArmController.h"

//---------------------------------------------------------------------------------------------------------------------
bool IndividualArmController::init(int _argc, char** _argv){

	if (_argc < 2) {
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
    
    std::this_thread::sleep_for(std::chrono::seconds(mConfigFile["id"].GetInt()*2));

    mActuateBackend = mConfigFile["enable_backend"].GetBool();
    mName = mConfigFile["name"].GetString();
    bool visualize = mConfigFile["visualize"].GetBool();

    ros::init(_argc, _argv, "individual_controller_"+mName);
    
    mRosSpinner = new ros::AsyncSpinner(4); // Use 4 threads
    mRosSpinner->start();

    hecatonquiros::Backend::Config bc; 
    if(mActuateBackend){ 
		bc.type = hecatonquiros::Backend::Config::eType::ROS; 
        bc.armId = mConfigFile["id"].GetInt();
        bc.configXML = mConfigFile["configXML"].GetString(); 
        bc.port = mConfigFile["serial_port"].GetString();
        bc.ndof =  mConfigFile["ndof"].GetInt();
    }else{
        bc.type = hecatonquiros::Backend::Config::eType::Dummy;
    }

    //arm controller
    hecatonquiros::ModelSolver::Config ms;
    ms.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    ms.robotName = mName;
    ms.manipulatorName = "manipulator";
    ms.robotFile = mConfigFile["arm_model"].GetString();
    std::vector<float> off;
    const rapidjson::Value& o = mConfigFile["offset"];
    for (auto& v : o.GetArray()){
        off.push_back(v.GetDouble());
    }
    ms.offset = off;
    ms.rotation = {0,0,0,1};
    ms.visualizer = visualize;
    mArm = new hecatonquiros::Arm4DoF(ms, bc);
    std::cout << "Arm created "+ mName << std::endl;

    mNdof = mArm->joints().size();
    // std::cout << "Manipulator has " << mNdof << " dof." << std::endl;

    if(mNdof == 3)
        cHomeJoints = {0,-M_PI/5,M_PI/1.5};
    if(mNdof == 4)
        cHomeJoints = {0,-M_PI/5,M_PI/1.5, 0};
        //cHomeJoints = {0,0, 90*M_PI/180.0, 0};
    if(mNdof == 6)
        cHomeJoints = {0,-M_PI/6,M_PI/1.5, -90*M_PI/180.0, /*-M_PI/1.5*/ 0,0};

    // ROS publishers and subscribers.
    ros::NodeHandle nh;
    
    mStatePublisher = nh.advertise<std_msgs::String>("/hecatonquiros/"+mName+"/out/controller/state", 1);  

    // Direct joints access
    mTargetJointsSubscriber = new WatchdogJoints("/hecatonquiros/"+mName+"/in/target_joints", 0.2);
    WatchdogJoints::Callback wrapperJointsCallback = [&](const typename sensor_msgs::JointState::ConstPtr &_msg){jointsCallback(_msg);};
    mTargetJointsSubscriber->attachCallback(wrapperJointsCallback);

    // Specific methods
    mTargetPoseLine3DSubscriber = new WatchdogPose("/hecatonquiros/"+mName+"/in/target_pose_line3d", 0.2);
    WatchdogPose::Callback wrapperPoseLine3dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){poseLine3DCallback(_msg);};
    mTargetPoseLine3DSubscriber->attachCallback(wrapperPoseLine3dCallback);

    // FAST-IK methods
    mTargetPose3DSubscriber = new WatchdogPose("/hecatonquiros/"+mName+"/in/target_pose3d", 0.2);
    WatchdogPose::Callback wrapperPose3dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){pose3DCallback(_msg);};
    mTargetPose3DSubscriber->attachCallback(wrapperPose3dCallback);

    mTargetPose4DSubscriber = new WatchdogPose("/hecatonquiros/"+mName+"/in/target_pose4d", 0.2);
    WatchdogPose::Callback wrapperPose4dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){pose4DCallback(_msg);};
    mTargetPose4DSubscriber->attachCallback(wrapperPose4dCallback);

    mTargetPose6DSubscriber = new WatchdogPose("/hecatonquiros/"+mName+"/in/target_pose6d", 0.2);
    WatchdogPose::Callback wrapperPose6dCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){pose6DCallback(_msg);};
    mTargetPose6DSubscriber->attachCallback(wrapperPose6dCallback);

    // Jacobian methods
    mTargetPose3DJacobiSubscriber = new WatchdogPose("/hecatonquiros/"+mName+"/in/target_pose3d_jacobi", 0.2);
    WatchdogPose::Callback wrapperPose3dJacobiCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){pose3DJacobiCallback(_msg);};
    mTargetPose3DJacobiSubscriber->attachCallback(wrapperPose3dJacobiCallback);

    mTargetPose6DJacobiSubscriber = new WatchdogPose("/hecatonquiros/"+mName+"/in/target_pose6d_jacobi", 0.2);
    WatchdogPose::Callback wrapperPose6dJacobiCallback = [&](const typename geometry_msgs::PoseStamped::ConstPtr &_msg){pose6DJacobiCallback(_msg);};
    mTargetPose6DJacobiSubscriber->attachCallback(wrapperPose6dJacobiCallback);

    mClawService = nh.advertiseService("/hecatonquiros/"+mName+"/in/claw", &IndividualArmController::clawService, this);
    mEmergencyStopService = nh.advertiseService("/hecatonquiros/"+mName+"/in/emergency_stop", &IndividualArmController::emergencyStopService, this);

    return true;
}


//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::start(){
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

    mMainThread = std::thread(&IndividualArmController::stateMachine, this);
    std::cout << "INDARMCONT: Created main thread" << std::endl;

    mJointsPublisherThread = std::thread(&IndividualArmController::publisherLoop, this);
    std::cout << "INDARMCONT: Created publisher thread" << std::endl;

}

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::stop(){
    mLastError = "Intentionally stoped";
    mState = STATES::STOP;
    mRunning = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if(mStatePublisherThread.joinable())    mStatePublisherThread.join();
    if(mMainThread.joinable())              mMainThread.join();

}

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::stateMachine(){

    mLastAimedJointsThread = std::thread(&IndividualArmController::MovingArmThread, this);

    while(ros::ok() && mRunning){
        switch(mState){
            case STATES::HOME:
                mTargetJoints = cHomeJoints;
                mLastAimedJoints = mTargetJoints;
                mArm->joints(cHomeJoints, mActuateBackend);
                break;
            case STATES::IDLE:
                mTargetJoints = cHomeJoints;
                mLastAimedJoints = mTargetJoints;
                mArm->joints(cHomeJoints, mActuateBackend);
                break;
            case STATES::STOP:
                mArm->openClaw();
                break;
            case STATES::MOVING:    
                break;
            case STATES::ERROR:
                break;
        }

        if( mTargetJointsSubscriber->isValid() ||
            mTargetPoseLine3DSubscriber->isValid() ||
            mTargetPose3DSubscriber->isValid() ||
            mTargetPose4DSubscriber->isValid() || 
            mTargetPose6DSubscriber->isValid() ||
            mTargetPose3DJacobiSubscriber->isValid() || 
            mTargetPose6DJacobiSubscriber->isValid() )
        {
            mState = STATES::MOVING;
        }else{		
            if(mState != STATES::HOME && mState != STATES::STOP){
                mArm->openClaw();
                mState = STATES::HOME;
            }
        }	

            //std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
}

//---------------------------------------------------------------------------------------------------------------------
bool IndividualArmController::clawService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res){
    if(_req.data){
        mArm->closeClaw();
    }else{
        mArm->openClaw();
    }

    _res.success = true;
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool IndividualArmController::emergencyStopService(std_srvs::SetBool::Request  &_req, std_srvs::SetBool::Response &_res){
    mEmergencyStop = _req.data;

    if(mEmergencyStop){
        mState = STATES::STOP;
    }

    _res.success = true;
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::MovingArmThread(){

    ros::Rate rate(30);

    auto moveIncLambda = [&](std::vector<float> _targetJoints)->std::vector<float>{  // INTEGRATE IN callback to limit speed
        
        std::vector<float> currJoints = mCurrJoints; // 666 thread safe?
        float MAX_JOINT_DIST = 20.0*M_PI/180.0; // 666 parametrize

        for(unsigned i = 0; i < _targetJoints.size();i++){
            float distJoint = (_targetJoints[i] - currJoints[i]);                  
            distJoint = distJoint > MAX_JOINT_DIST ? MAX_JOINT_DIST:distJoint;
            _targetJoints[i] = currJoints[i] + distJoint;
        }

        mArm->joints(_targetJoints, mActuateBackend);
        return _targetJoints;
    };

    while(mRunning && ros::ok()){
        
        switch(mState){
            case STATES::MOVING:
            {   
                mLastAimedJoints = moveIncLambda(mTargetJoints);
                rate.sleep();
            }
                break;
            default:
                rate.sleep();
                break;
        }
    }

}

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::publisherLoop(){
    ros::NodeHandle nh;
    ros::Publisher jointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/"+mName+"/out/joints_state", 1);
    ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/"+mName+"/out/pose", 1);
    ros::Publisher aimingJointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/"+mName+"/out/aiming_joints", 1);  
    ros::Publisher targetJointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/"+mName+"/out/target_joints", 1);

    ros::Rate rate(50);

    while(ros::ok()){
        // Publish target joints
        std::vector<float> targetJoints;
        sensor_msgs::JointState targetJointsMsg;
	    targetJointsMsg.header.stamp = ros::Time::now();
        targetJoints = mTargetJoints;
        for(auto&j:targetJoints){
            targetJointsMsg.position.push_back(j);
        }
        targetJointsPublisher.publish(targetJointsMsg);

        // Publish state joints
        std::vector<float> joints;
        sensor_msgs::JointState jointsMsg;
	    jointsMsg.header.stamp = targetJointsMsg.header.stamp;
        mCurrJoints = mArm->joints();
        joints = mCurrJoints;
        for(auto&j:joints){
            jointsMsg.position.push_back(j);
        }
        jointsPublisher.publish(jointsMsg);

        // Publish aiming joints
        sensor_msgs::JointState aimingJointsMsg;
	    aimingJointsMsg.header.stamp = targetJointsMsg.header.stamp;
        std::vector<float> aimJoints = mLastAimedJoints;
        for(auto&j:aimJoints){
            aimingJointsMsg.position.push_back(j);
        }
        aimingJointsPublisher.publish(aimingJointsMsg);

        // Publish current pose
        mCurrentPose = mArm->pose();
        geometry_msgs::PoseStamped poseMsg;
        eigenToRos(mCurrentPose, poseMsg);
        poseMsg.header.frame_id = "hecatonquiros_"+mName;
        poseMsg.header.stamp = targetJointsMsg.header.stamp;
        posePublisher.publish(poseMsg);

        rate.sleep();
    }

    std::cout << "INDARMCONT: Publisher Loop finished" << std::endl;
}   

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose){
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
void IndividualArmController::eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg){
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
void IndividualArmController::jointsCallback(const sensor_msgs::JointState::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            std::vector<float> joints;
            for(auto j:_msg->position){
                joints.push_back(j);
            }
            mTargetJoints = joints; // 666 Thread safe?
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::poseLine3DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){	   

            Eigen::Matrix4f finalPose;
            rosToEigen(_msg, finalPose);

            Eigen::Matrix4f pose = mCurrentPose;

            float dx = finalPose(0,3) - pose(0,3);
            float dy = finalPose(1,3) - pose(1,3);
            float dz = finalPose(2,3) - pose(2,3);

            Eigen::Vector3f dir = {dx, dy, dz};
            dir /=dir.norm();
            
            float distance = sqrt(dx*dx + dy*dy + dz*dz);
            float step_size = distance/2;   // 666 PROBLEMS WITH STEP SIZE WHEN DISTANCE IS TOO SHORT

            pose.block<3,1>(0,3) +=dir*step_size;

            std::vector<float> joints;
            if(mArm->checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
                mTargetJoints = joints; // 666 Thread safe?		
            }else{
                std::cout << "Failed IK" << std::endl;
            }
                
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::pose3DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){	    
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);

            std::vector<float> joints;
            if(mArm->checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
                mTargetJoints = joints; // 666 Thread safe?
            }else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::pose4DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
            if(mState == STATES::MOVING){	    
                Eigen::Matrix4f pose;
                rosToEigen(_msg, pose);

            std::vector<float> joints;
            if(mArm->checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){  
                Eigen::Vector3f a = pose.block<3,1>(0,0);
                Eigen::Vector3f b = -Eigen::Vector3f::UnitZ();
                Eigen::Vector3f c = Eigen::Vector3f::UnitX();
                float angle = acos(a.dot(b));
                int dir = (b.cross(a)).dot(c) < 0 ? 1:-1;

                std::vector<float> targetjoints = joints;
                targetjoints[3] = dir*angle;
                mTargetJoints = targetjoints; // 666 Thread safe?

            }else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::pose6DCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);
                
            std::vector<float> joints;
            if(mArm->checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
                mTargetJoints = joints; // 666 Thread safe?
            }  else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::pose3DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);

            std::vector<float> joints;
            Eigen::Vector3f position = pose.block<3,1>(0,3);
            if(mArm->jacobianStep(position, joints, 0.2)){
                mTargetJoints = joints; // 666 Thread safe?
            }else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------
void IndividualArmController::pose6DJacobiCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg){
        if(mState == STATES::MOVING){
            Eigen::Matrix4f pose;
            rosToEigen(_msg, pose);
                
            std::vector<float> joints;
            if(mArm->jacobianStep(pose, joints)){
                mTargetJoints = joints; // 666 Thread safe?
            }  else{
                std::cout << "Failed IK" << std::endl;
            }
        }
    }



