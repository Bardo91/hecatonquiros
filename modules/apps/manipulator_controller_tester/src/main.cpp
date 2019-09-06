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

#include <stdlib.h>     /* srand, rand */

void rosToEigen(const geometry_msgs::PoseStamped::ConstPtr &_msg, Eigen::Matrix4f &_pose);
void eigenToRos(Eigen::Matrix4f &_pose, geometry_msgs::PoseStamped &_msg);

int main(int _argc, char **_argv){
	ros::init(_argc, _argv, "ros_controller_tester");

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle nh;
	char test;
	bool finish = true;
	while(finish && ros::ok()){
		std::cout << "Circle Test -> c" << std::endl;
		std::cout << "Line test with some WP -> l" << std::endl;
		std::cout << "Fix Point Test -> f" << std::endl;
		std::cout << "Some Points Test -> p" << std::endl;
		std::cout << "Some Joints Test -> j" << std::endl;
		std::cout << "Emergency Service Test -> e" << std::endl;
		std::cout << "Open/Close Gripper -> g" << std::endl;
		std::cout << "Finish program -> q" << std::endl;
		std::cin >> test;
		switch(test){
			case 'q':
				finish = false;
				break;
			case 'c':
			{	
				std::cout << "Time(ms): ";
				int time;
				std::cin >> time;

				std::cout << "Vel1: ";
				int vel1;
				std::cin >> vel1;

				std::cout << "Vel2: ";
				int vel2;
				std::cin >> vel2;

				Eigen::Matrix4f leftPose;
				Eigen::Matrix4f rightPose;
				
				ros::Subscriber leftPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/out/pose", 1, 
							[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
								rosToEigen(_msg, leftPose);
							});
				ros::Subscriber rightPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/out/pose", 1, 
							[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
								rosToEigen(_msg, rightPose);
							});

				ros::Publisher leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/in/target_pose3d", 1);
				ros::Publisher rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/in/target_pose3d", 1);

				std::cout << "Wait for pose" << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));

				Eigen::Matrix4f leftInitPose = leftPose, rightInitPose = rightPose;
				Eigen::Matrix4f leftSecondPose = leftPose, rightSecondPose = rightPose;
				leftInitPose(2,3) += +0.03;
				leftSecondPose(2,3) += +0.03;
				leftSecondPose(2,3) += +0.03;
				rightSecondPose(2,3) += +0.03;

				std::cout << "Circle TEST" << std::endl;
				
				auto t0 = std::chrono::high_resolution_clock::now();
				float duration = 0;
				while(duration < time && ros::ok()){

					std::this_thread::sleep_for(std::chrono::milliseconds(10));
					auto t1 = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();

					float incY = cos(2*M_PI*duration/vel1)*0.07;
					float incZ = sin(2*M_PI*duration/vel2)*0.07;

					Eigen::Matrix4f leftTargetPose, rightTargetPose;
					if(duration < time/2){
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
			case 'l':
			{
				ros::Publisher leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/in/target_pose_line3d", 1);
				ros::Publisher rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/in/target_pose_line3d", 1);

				Eigen::Matrix4f leftPose;
				Eigen::Matrix4f rightPose;
				
				ros::Subscriber leftPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/out/pose", 1, 
							[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
								rosToEigen(_msg, leftPose);
							});
				ros::Subscriber rightPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/out/pose", 1, 
							[&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
								rosToEigen(_msg, rightPose);
							});

				std::cout << "Wait for pose" << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));

				Eigen::Matrix4f leftInitPose = leftPose, rightInitPose = rightPose;

				std::vector<Eigen::Matrix4f> leftArm, rightArm;

				/* initialize random seed: */
				srand (time(NULL));

				std::cout << "How many points rand you want: ";
				int npoints;
				std::cin >> npoints;
				auto randf01 = [](){ return ((float)rand())/RAND_MAX;};

				for(unsigned i = 0; i < npoints; i ++){
						float numbRandXY = randf01() * 0.2;
						float numbRandZ = randf01() * 0.1;

						Eigen::Matrix4f pose = rightInitPose;
						pose(0,3) += numbRandXY;
						pose(1,3) += numbRandXY;
						pose(2,3) += numbRandZ;
						rightArm.push_back(pose);

						pose(1,3) = -pose(1,3);
						leftArm.push_back(pose);
	
						
				}
				
				auto t0 = std::chrono::high_resolution_clock::now();
				float duration = 0;
				float cont = 0;

				std::cout << "Time(ms): ";
				int time;
				std::cin >> time;
				// int time = 50000;

				int i = 0;

				cont = 3000;
				while(duration < time && ros::ok()){

					std::cout << "Moving to:  " << rightArm[i](0,3) << "  " << rightArm[i](1,3) << "  " << rightArm[i](2,3) << std::endl; 

					geometry_msgs::PoseStamped leftMsg;	
					eigenToRos(leftArm[i], leftMsg);
					leftMsg.header.stamp = ros::Time::now();

					geometry_msgs::PoseStamped rightMsg;
					eigenToRos(rightArm[i], rightMsg);
					rightMsg.header.stamp = ros::Time::now();

					i++;
					if(i >= npoints){
						i = 0;
					}

					while(duration < cont && ros::ok()){
						
						leftPosePublisher.publish(leftMsg);
						rightPosePublisher.publish(rightMsg);
						std::this_thread::sleep_for(std::chrono::milliseconds(100));

						auto t1 = std::chrono::high_resolution_clock::now();
						duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
					}
				
					cont = cont + 3000;
				}				
				break;
			}
			case 'p':
			{
				ros::Publisher leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/in/target_pose3d", 1);
				ros::Publisher rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/in/target_pose3d", 1);

				auto t0 = std::chrono::high_resolution_clock::now();
				float duration = 0;
				float incX = 0.25, incY = 0.15, incZ = 0.35;

				std::cout << "Time(ms): ";
				int time;
				std::cin >> time;

				while(duration < time && ros::ok()){

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
			case 'f':
			{	
				std::cout << "Pose3d -> 1, PoseLine3d -> 2" << std::endl; 
				int select;
				std::cin >> select;

				ros::Publisher leftPosePublisher;
				ros::Publisher rightPosePublisher; 
				if(select == 1){
					leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/in/target_pose3d", 1);
					rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/in/target_pose3d", 1);
				}else if(select == 2){
					leftPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/left_arm/in/target_pose_line3d", 1);
					rightPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/in/target_pose_line3d", 1);
				}else{
					std::cout << "Bad select" << std::endl; 
					break;
				}
				

				float x, y, z;
				std::cout << "X: ";
				std::cin >> x;
				std::cout << "Y: ";
				std::cin >> y;
				std::cout << "Z: ";
				std::cin >> z;

				while(ros::ok()){
					geometry_msgs::PoseStamped leftMsg;
					leftMsg.pose.position.x = x;
					leftMsg.pose.position.y = -y;
					leftMsg.pose.position.z =  z;
					leftMsg.pose.orientation.x = 0;
					leftMsg.pose.orientation.y = 0;
					leftMsg.pose.orientation.z = 0;
					leftMsg.pose.orientation.w = 1;
					leftMsg.header.stamp = ros::Time::now();

					geometry_msgs::PoseStamped rightMsg;
					rightMsg.pose.position.x = x;
					rightMsg.pose.position.y = y;
					rightMsg.pose.position.z = z;
					rightMsg.pose.orientation.x = 0;
					rightMsg.pose.orientation.y = 0;
					rightMsg.pose.orientation.z = 0;
					rightMsg.pose.orientation.w = 1;
					rightMsg.header.stamp = ros::Time::now();

					leftPosePublisher.publish(leftMsg);
					rightPosePublisher.publish(rightMsg);

					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}			
				break;
			}
			case 'j':
			{
				ros::Publisher leftJointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/left_arm/in/target_joints", 1);
				ros::Publisher rightJointsPublisher = nh.advertise<sensor_msgs::JointState>("/hecatonquiros/right_arm/in/target_joints", 1);													

				auto t0 = std::chrono::high_resolution_clock::now();
				float duration = 0;
				int incA = 90;

				std::cout << "Time(ms): ";
				int time;
				std::cin >> time;
				
				while(duration < time && ros::ok()){

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
				ros::ServiceClient leftEmergServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/left_arm/in/emergency_stop");
				ros::ServiceClient rightEmergServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/right_arm/in/emergency_stop");
				
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
				ros::ServiceClient leftGripServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/left_arm/in/claw");
				ros::ServiceClient rightGripServ = nh.serviceClient<std_srvs::SetBool>("/hecatonquiros/right_arm/in/claw");
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