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
	char test;
	while(true){
		std::cout << "Circle Test -> c" << std::endl;
		std::cout << "Some Points Test -> p" << std::endl;
		std::cout << "Some Joints Test -> j" << std::endl;
		std::cout << "Emergency Service Test -> e" << std::endl;
		std::cout << "Open/Close Gripper -> g" << std::endl;
		std::cin >> test;
		switch(test){
			case 'c':
			{
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
				Eigen::Matrix4f leftSecondPose = leftPose, rightSecondPose = rightPose;
				leftSecondPose(2,3) += -0.15;
				rightSecondPose(2,3) += -0.15;

				std::cout << "Circle TEST" << std::endl;
				
				auto t0 = std::chrono::high_resolution_clock::now();
				float duration = 0;
				while(duration < 50000){

					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					auto t1 = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

					float incY = cos(2*M_PI*duration/10000)*0.1;
					float incZ = sin(2*M_PI*duration/10000)*0.1;

					Eigen::Matrix4f leftTargetPose, rightTargetPose;
					if(duration < 25000){
						leftTargetPose = leftInitPose;
						rightTargetPose = rightInitPose;		
					}else{
						leftTargetPose = leftSecondPose;
						rightTargetPose = rightSecondPose;
					}
					
					leftTargetPose(1,3) += incY;
					leftTargetPose(2,3) += incZ;
					geometry_msgs::PoseStamped leftMsg;
					eigenToRos(leftTargetPose, leftMsg);
					leftMsg.header.stamp = ros::Time::now();

					rightTargetPose(1,3) += incY;
					rightTargetPose(2,3) += incZ;
					geometry_msgs::PoseStamped rightMsg;
					eigenToRos(rightTargetPose, rightMsg);
					rightMsg.header.stamp = ros::Time::now();

					leftPosePublisher.publish(leftMsg);
					rightPosePublisher.publish(rightMsg);	
				}
				break;
			}
			case 'p':
			{
				ros::Publisher leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/target_pose3d", 1);
				ros::Publisher rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/target_pose3d", 1);

				auto t0 = std::chrono::high_resolution_clock::now();
				float duration = 0;
				float incX = 0.29, incY = 0.15, incZ = 0.29;

				while(duration < 5000){

					incX += 0;
					incY += 0.01;
					incZ += 0;

					geometry_msgs::PoseStamped leftMsg;
					leftMsg.pose.position.x = incX;
					leftMsg.pose.position.y = -incY;
					leftMsg.pose.position.z =  incZ;
					leftMsg.pose.orientation.x = 0;
					leftMsg.pose.orientation.y = 0;
					leftMsg.pose.orientation.z = 0;
					leftMsg.pose.orientation.w = 1;
					leftMsg.header.stamp = ros::Time::now();

					geometry_msgs::PoseStamped rightMsg;
					rightMsg.pose.position.x = incX;
					rightMsg.pose.position.y = incY;
					rightMsg.pose.position.z = incZ;
					rightMsg.pose.orientation.x = 0;
					rightMsg.pose.orientation.y = 0;
					rightMsg.pose.orientation.z = 0;
					rightMsg.pose.orientation.w = 1;
					rightMsg.header.stamp = ros::Time::now();

					leftPosePublisher.publish(leftMsg);
					rightPosePublisher.publish(rightMsg);

					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					auto t1 = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

				}				
				break;
			}
			case 'j':
			{
				ros::Publisher leftJointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/left_arm/target_joints", 1);
				ros::Publisher rightJointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/right_arm/target_joints", 1);													

				auto t0 = std::chrono::high_resolution_clock::now();
				float duration = 0;
				int incA = 90;
				while(duration < 5000){

					incA = incA - 5;

					sensor_msgs::JointState leftMsg;	
					leftMsg.position.push_back(0.0*M_PI/180.0);
					leftMsg.position.push_back(0.0*M_PI/180.0);
					leftMsg.position.push_back(incA*M_PI/180.0);
					leftMsg.position.push_back(0.0*M_PI/180.0);
					leftMsg.header.stamp = ros::Time::now();
				
					sensor_msgs::JointState rightMsg;
					rightMsg.position.push_back(0.0*M_PI/180.0);
					rightMsg.position.push_back(0.0*M_PI/180.0);
					rightMsg.position.push_back(incA*M_PI/180.0);
					rightMsg.position.push_back(0.0*M_PI/180.0);
					rightMsg.header.stamp = leftMsg.header.stamp;

					leftJointsPublisher.publish(leftMsg);
					rightJointsPublisher.publish(rightMsg);	

					std::this_thread::sleep_for(std::chrono::milliseconds(200));
					auto t1 = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

				}	

				break;
			}
			case 'e':
			{
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
				break;
			}
			case 'g':
			{
				ros::ServiceClient leftGripServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/left_arm/claw");
				ros::ServiceClient rightGripServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/right_arm/claw");
				std_srvs::SetBool srv;

				std::cout << "1 close gripper, 2 open gripper" << std::endl;
				int grip;
				std::cin >> grip;
				if(grip == 1){
					srv.request.data = true;
				}else if(grip == 2){
					srv.request.data = false;
				}else{
					std::cout << "Number error!" << std::endl;
				}
				if(leftGripServ.call(srv)){
					if(srv.response.success){
						std::cout << "response success left is TRUE" << std::endl;
					}else{
						std::cout << "response success left is FALSE" << std::endl;
					}
				}
				if(rightGripServ.call(srv)){
					if(srv.response.success){
						std::cout << "response success right is TRUE" << std::endl;
					}else{
						std::cout << "response success right is FALSE" << std::endl;
					}
				}
				break;
			}
		}
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