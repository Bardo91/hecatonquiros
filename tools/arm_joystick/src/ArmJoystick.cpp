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


#include "ArmJoystick.h"

//---------------------------------------------------------------------------------------------------------------------
bool ArmJoystick::init(int _argc, char** _argv){

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

	mLeftArmPub = mConfigFile["leftarmpub"].GetString();
	mRightArmPub = mConfigFile["rightarmpub"].GetString();

	mLeftArmSub = mConfigFile["leftarmsub"].GetString();
	mRightArmSub = mConfigFile["rightarmsub"].GetString();

	mJoySub = mConfigFile["joysub"].GetString();

	mVel = mConfigFile["vel"].GetDouble();

	mJoyAxes.resize(7);
	mJoyButtons.resize(12);	

	//////////////////////////////////////////////////
	ros::NodeHandle nh;
	mPoseLeftArmSubscriber = nh.subscribe<geometry_msgs::PoseStamped>(mLeftArmSub, 1, \
	[&](const geometry_msgs::PoseStamped::ConstPtr &_msg) {
		rosToEigen(_msg, mPoseLeft);
	});

	mPoseRightArmSubscriber = nh.subscribe<geometry_msgs::PoseStamped>(mRightArmSub, 1, \
	[&](const geometry_msgs::PoseStamped::ConstPtr &_msg) {
		rosToEigen(_msg, mPoseRight);
	});
	std::cout << "Created ROS Subscribers" << std::endl;

	//////////////////////////////////////////////////
	mTargetLeftArmPublisher = nh.advertise<geometry_msgs::PoseStamped>(mLeftArmPub, 1);
	mTargetRightArmPublisher = nh.advertise<geometry_msgs::PoseStamped>(mRightArmPub, 1);
	std::cout << "Created ROS Publishers" << std::endl;

	std::cout << "Wait for pose" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	mCurrentPoseLeft = mPoseLeft;
	mCurrentPoseRight = mPoseRight;
	std::cout << "Pose Left: " << std::endl;
	std::cout << mCurrentPoseLeft << std::endl;
	std::cout << "Pose Right: " << std::endl;
	std::cout << mCurrentPoseRight << std::endl;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool ArmJoystick::start(){

    mRun = true;
	mJoyThread = std::thread(&ArmJoystick::joystickThread, this);
    mLoopThread = std::thread(&ArmJoystick::loop, this);   

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool ArmJoystick::stop(){
    
	std::cout << "Stop program" << std::endl;
    mRun = false;
    if(mLoopThread.joinable()) mLoopThread.join();
	if(mJoyThread.joinable()) mJoyThread.join();

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void ArmJoystick::loop(){

	std::cout << "Started Loop thread" << std::endl;
	while(ros::ok() && mRun){
		switch(mState){
			case STATES::STOP:
				stopFunction();
				break;
			case STATES::MOVING:
				movingFunction();
				break;
			case STATES::ERROR:
				break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}	
}

//---------------------------------------------------------------------------------------------------------------------
void ArmJoystick::movingFunction(){

	if(mSecButLeft){
		mCurrentPoseLeft(0,3) += mVelX1;
		mCurrentPoseLeft(1,3) += mVelY1;
		mCurrentPoseLeft(2,3) += mVelZ1; 
		geometry_msgs::PoseStamped poseMsgLeft;
		eigenToRos(mCurrentPoseLeft, poseMsgLeft);
		poseMsgLeft.header.stamp = ros::Time::now();
		mTargetLeftArmPublisher.publish(poseMsgLeft);
	}
	
	if(mSecButRight){
		mCurrentPoseRight(0,3) += mVelX2;
		mCurrentPoseRight(1,3) += mVelY2;
		if(mVelZ2Up){
			mCurrentPoseRight(2,3) += mVelZ2Up;
		}
		if(mVelZ2Down){
			mCurrentPoseRight(2,3) += mVelZ2Down;
		}
		geometry_msgs::PoseStamped poseMsgRight;
		eigenToRos(mCurrentPoseRight, poseMsgRight);
		poseMsgRight.header.stamp = ros::Time::now();
		mTargetRightArmPublisher.publish(poseMsgRight);
	}
	
}

//---------------------------------------------------------------------------------------------------------------------
void ArmJoystick::stopFunction(){

	geometry_msgs::PoseStamped poseMsgLeft;
	eigenToRos(mCurrentPoseLeft, poseMsgLeft);
	poseMsgLeft.header.stamp = ros::Time::now();
	mTargetLeftArmPublisher.publish(poseMsgLeft);

	geometry_msgs::PoseStamped poseMsgRight;
	eigenToRos(mCurrentPoseRight, poseMsgRight);
	poseMsgRight.header.stamp = ros::Time::now();
	mTargetRightArmPublisher.publish(poseMsgRight);

}

//---------------------------------------------------------------------------------------------------------------------
void ArmJoystick::joystickThread(){

	std::cout << "Started Joystick thread" << std::endl;

	ros::NodeHandle nh;
    mJoySubscriber = nh.subscribe<sensor_msgs::Joy>(mJoySub, 1, \
	[&](const sensor_msgs::Joy::ConstPtr &_msg) {
		
		mJoyAxes = _msg->axes;
		mJoyButtons = _msg->buttons;

	});

	while(ros::ok() && mRun){
		// AXIS X1
		if(mJoyAxes[0] == 1){
			mVelX1 += mVel;
		}else if(mJoyAxes[0] == 0){
			mVelX1 = 0;
		}else if(mJoyAxes[0] == -1){
			mVelX1 -= mVel;
		}else{
			std::cout << "Unknown value X1 of joystick" << std::endl;
		}

		// AXIS Y1
		if(mJoyAxes[1] == 1){
			mVelY1 += mVel;
		}else if(mJoyAxes[1] == 0){
			mVelY1 = 0;
		}else if(mJoyAxes[1] == -1){
			mVelY1 -= mVel;
		}else{
			std::cout << "Unknown value Y1 of joystick" << std::endl;
		}

		// AXIS X2
		if(mJoyAxes[3] == 1){
			mVelX2 += mVel;
		}else if(mJoyAxes[3] == 0){
			mVelX2 = 0;
		}else if(mJoyAxes[3] == -1){
			mVelX2 -= mVel;
		}else{
			std::cout << "Unknown value X2 of joystick" << std::endl;
		}

		// AXIS Y2
		if(mJoyAxes[4] == 1){
			mVelY2 += mVel;
		}else if(mJoyAxes[4] == 0){
			mVelY2 = 0;
		}else if(mJoyAxes[4] == -1){
			mVelY2 -= mVel;
		}else{
			std::cout << "Unknown value Y2 of joystick" << std::endl;
		}

		// AXIS Z1
		if(mJoyAxes[6] == 1){
			mVelZ1 += mVel;
		}else if(mJoyAxes[6] == 0){
			mVelZ1 = 0;
		}else if(mJoyAxes[6] == -1){
			mVelZ1 -= mVel;
		}else{
			std::cout << "Unknown value Z1 of joystick" << std::endl;
		}

		// AXIS Z2
		if(mJoyButtons[0] == 1){
			mVelZ2Up += mVel;
		}else if(mJoyButtons[0] == 0){
			mVelZ2Up = 0;
		}else{
			std::cout << "Unknown value Z2+ of joystick" << std::endl;
		}

		if(mJoyButtons[2] == 1){
			mVelZ2Down -= mVel;
		}else if(mJoyButtons[2] == 0){
			mVelZ2Down = 0;
		}else{
			std::cout << "Unknown value Z2- of joystick" << std::endl;
		}

		// SECURITY BUTTON LEFT
		if(mJoyButtons[4] == 1){
			mSecButLeft = true;
		}else if(mJoyButtons[4] == 0){
			mSecButLeft = false;
		}else{
			std::cout << "Unknown value Security button left of joystick" << std::endl;
		}

		// SECURITY BUTTON RIGHT
		if(mJoyButtons[5] == 1){
			mSecButRight = true;
		}else if(mJoyButtons[5] == 0){
			mSecButRight = false;
		}else{
			std::cout << "Unknown value Security button right of joystick" << std::endl;
		}

		// UPDATE POSE
		if(mJoyButtons[9] == 1){
			mCurrentPoseLeft = mPoseLeft;
			mCurrentPoseRight = mPoseRight;
			std::cout << "Pose Left: " << std::endl;
			std::cout << mCurrentPoseLeft << std::endl;
			std::cout << "Pose Right: " << std::endl;
			std::cout << mCurrentPoseRight << std::endl;
		}

		if(mVelX1 || mVelX2 || mVelY1 || mVelY2 || mVelZ1 || mVelZ2Up || mVelZ2Down){
			mState = STATES::MOVING;
		}else{
			mState = STATES::STOP;
		} 

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }



}

//---------------------------------------------------------------------------------------------------------------------
void ArmJoystick::rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose){
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
void ArmJoystick::eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg){
	_msg.pose.position.x = _pose(0,3);
	_msg.pose.position.y = _pose(1,3);
	_msg.pose.position.z = _pose(2,3);

	Eigen::Quaternionf q(_pose.block<3,3>(0,0));
	_msg.pose.orientation.x = q.x();
	_msg.pose.orientation.y = q.y();
	_msg.pose.orientation.z = q.z();
	_msg.pose.orientation.w = q.w();

}

