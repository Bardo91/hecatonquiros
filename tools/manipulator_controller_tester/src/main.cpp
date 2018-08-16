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
#include <Eigen/Eigen>

void rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose);
void eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg);

int main(int _argc, char **_argv){
	ros::init(_argc, _argv, "ros_controller_tester");

	ros::NodeHandle nh;

	

	ros::Subscriber poseLeftSubscriber = nh.subscribe("/hecatonquiros/left/pose",1,[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
		
	});
	ros::Subscriber poseRightSubscriber = nh.subscribe("/hecatonquiros/right/pose",1,[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
		
	});


	
	ros::Subscriber jointsLeftPublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left/pose",1);
	ros::Subscriber jointsRighPublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right/pose",1);
	
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

}