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
#include <std_srvs/SetBool.h>
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

	std::cout << "Circle Test -> 1" << std::endl;
	std::cout << "Point Test -> 2" << std::endl;
	std::cout << "Joints Test -> 3" << std::endl;
	std::cout << "Emergency Service Test -> 4" << std::endl;
	int test;
	std::cin >> test;

	if(test == 1){

		Eigen::Matrix4f leftPose;
		Eigen::Matrix4f rightPose;
		
		ros::Subscriber leftPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/pose", 1, 
					[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
						rosToEigen(_msg, leftPose);
					});
		ros::Subscriber rightPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/pose", 1, 
					[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
						rosToEigen(_msg, rightPose);
					});

		ros::Publisher leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/target_pose3d_jacobi", 1);
		ros::Publisher rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/target_pose3d_jacobi", 1);

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
	}else if(test == 2){

		ros::Publisher leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/target_pose3d", 1);
		ros::Publisher rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/target_pose3d", 1);
		
		while(ros::ok()){

			geometry_msgs::PoseStamped leftMsg;
			leftMsg.pose.position.x = 0.29;
			leftMsg.pose.position.y = -0.15;
			leftMsg.pose.position.z = 0.29;
			leftMsg.pose.orientation.x = 0;
			leftMsg.pose.orientation.y = 0;
			leftMsg.pose.orientation.z = 0;
			leftMsg.pose.orientation.w = 1;
			leftMsg.header.stamp = ros::Time::now();

			geometry_msgs::PoseStamped rightMsg;
			rightMsg.pose.position.x = 0.29;
			rightMsg.pose.position.y = 0.15;
			rightMsg.pose.position.z = 0.29;
			rightMsg.pose.orientation.x = 0;
			rightMsg.pose.orientation.y = 0;
			rightMsg.pose.orientation.z = 0;
			rightMsg.pose.orientation.w = 1;
			rightMsg.header.stamp = ros::Time::now();

			leftPosePublisher.publish(leftMsg);
			rightPosePublisher.publish(rightMsg);
		}
	}else if(test == 3){
		ros::Publisher leftPosePublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/left_arm/target_joints", 1);
		ros::Publisher rightPosePublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/right_arm/target_joints", 1);
		
		while(ros::ok()){
			
			sensor_msgs::JointState leftMsg;
			leftMsg.position.push_back(0.0*M_PI/180.0);
			leftMsg.position.push_back(0.0*M_PI/180.0);
			leftMsg.position.push_back(90.0*M_PI/180.0);
			leftMsg.position.push_back(0.0*M_PI/180.0);
			leftMsg.header.stamp = ros::Time::now();

			sensor_msgs::JointState rightMsg;
			rightMsg.position.push_back(0.0*M_PI/180.0);
			rightMsg.position.push_back(0.0*M_PI/180.0);
			rightMsg.position.push_back(90.0*M_PI/180.0);
			rightMsg.position.push_back(0.0*M_PI/180.0);
			rightMsg.header.stamp = leftMsg.header.stamp;

			leftPosePublisher.publish(leftMsg);
			rightPosePublisher.publish(rightMsg);
		}
	}else if(test == 4){
		ros::ServiceClient leftEmergServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/left_arm/emergency_stop");
		ros::ServiceClient rightEmergServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/right_arm/emergency_stop");
		
		
		std_srvs::SetBool srv;
		srv.request.data = true;
		if(leftEmergServ.call(srv)){
			if(srv.response.success){
				std::cout << "response success left is TRUE" << std::endl;
			}
		}else{
			std::cout << "response success left is FALSE" << std::endl;
		}

		if(rightEmergServ.call(srv)){
			if(srv.response.success){
				std::cout << "response success right is TRUE" << std::endl;
			}
		}else{
			std::cout << "response success right is FALSE" << std::endl;
		}

	}else{
		std::cout << "Unknown Test!" << std::endl;
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