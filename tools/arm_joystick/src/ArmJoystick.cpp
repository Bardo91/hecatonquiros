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






    // FALTA PONER SUBSCRIPTOR A JOY, Y QUE SEA CONTINUO EN VELOCIDADES 









    ros::NodeHandle nh;

    ros::Publisher targetLeftArmPublisher = nh.advertise<geometry_msgs::PoseStamped>(_argv[2], 1);
	ros::Publisher targetRightArmPublisher = nh.advertise<geometry_msgs::PoseStamped>(_argv[3], 1);
	std::cout << "Created ROS Publishers" << std::endl;

	Eigen::Matrix4f poseLeft;
	Eigen::Matrix4f poseRight;

	ros::Subscriber poseLeftArmSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/out/pose", 1, \
	[&](const geometry_msgs::PoseStamped::ConstPtr &_msg) {
		rosToEigen(_msg, poseLeft);
	});

	ros::Subscriber poseRightArmSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/out/pose", 1, \
	[&](const geometry_msgs::PoseStamped::ConstPtr &_msg) {
		rosToEigen(_msg, poseRight);
	});
	std::cout << "Created ROS Subscribers" << std::endl;

	std::cout << "Wait for pose" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	currentPoseLeft = poseLeft;
	currentPoseRight = poseRight;

	std::cout << "currentPoseLeft: \n" << currentPoseLeft << std::endl;
	std::cout << "currentPoseRight: \n" << currentPoseRight << std::endl;

    std::thread joyThread = std::thread(joystickThread, _argv[1]);


    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool ArmJoystick::start(){

    mRun = true;
    mLoopThread = std::thread(&ArmJoystick::loop, this);   

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool ArmJoystick::stop(){
    
    mRun = false;
    if(mLoopThread.joinable()) mLoopThread.join();

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
void ArmJoystick::loop(){
    while(ros::ok() && mRun){
        geometry_msgs::PoseStamped leftMsg;
        leftMsg.header.stamp = ros::Time::now();
        eigenToRos(currentPoseLeft, leftMsg);

        geometry_msgs::PoseStamped rightMsg;
        rightMsg.header.stamp = ros::Time::now();
        eigenToRos(currentPoseRight, rightMsg);

        targetLeftArmPublisher.publish(leftMsg);
        targetRightArmPublisher.publish(rightMsg);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

//---------------------------------------------------------------------------------------------------------------------
void ArmJoystick::joystickThread(std::string _joy){

    

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

