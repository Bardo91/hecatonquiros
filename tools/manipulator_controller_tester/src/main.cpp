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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Eigen>
#include <chrono>
#include <thread>

void rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose);
void eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg);

int main(int _argc, char **_argv){
	ros::init(_argc, _argv, "ros_controller_tester");

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle nh;
	
	Eigen::Matrix4f leftPose;
	Eigen::Matrix4f rightPose;
	
	ros::Subscriber leftPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/left/pose", 1, 
				[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
					rosToEigen(_msg, leftPose);
				});
	ros::Subscriber rightPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/right/pose", 1, 
				[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
					rosToEigen(_msg, rightPose);
				});

	ros::Publisher leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left/target_pose3d_jacobi", 1);
	ros::Publisher rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right/target_pose3d_jacobi", 1);

	std::cout << "Wait for pose" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	Eigen::Matrix4f leftInitPose = leftPose, rightInitPose = rightPose;


	std::cout << "Circle TEST" << std::endl;
	
	auto t0 = std::chrono::high_resolution_clock::now();
	float duration = 0;
	while(duration < 50000){

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		auto t1 = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

		float incY = cos(2*M_PI*duration/10000)*0.1;
		float incZ = sin(2*M_PI*duration/10000)*0.1;

		Eigen::Matrix4f leftTargetPose = leftInitPose;
		leftTargetPose(1,3) += incY;
		leftTargetPose(2,3) += incZ;
		geometry_msgs::PoseStamped leftMsg;
		eigenToRos(leftTargetPose, leftMsg);
		leftMsg.header.stamp = ros::Time::now();

		Eigen::Matrix4f rightTargetPose = rightInitPose;
		rightTargetPose(1,3) += incY;
		rightTargetPose(2,3) += incZ;
		geometry_msgs::PoseStamped rightMsg;
		eigenToRos(rightTargetPose, rightMsg);
		rightMsg.header.stamp = ros::Time::now();

		leftPosePublisher.publish(leftMsg);
		rightPosePublisher.publish(rightMsg);
		
	}

}



//---------------------------------------------------------------------------------------------------------------------
void rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose){
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
void eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg){
	_msg.pose.position.x = _pose(0,3);
	_msg.pose.position.y = _pose(1,3);
	_msg.pose.position.z = _pose(2,3);

	Eigen::Quaternionf q(_pose.block<3,3>(0,0));
	_msg.pose.orientation.x = q.x();
	_msg.pose.orientation.y = q.y();
	_msg.pose.orientation.z = q.z();
	_msg.pose.orientation.w = q.w();
}