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


#include <Eigen/Eigen>
#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


geometry_msgs::PoseStamped eigen2PoseStamped(Eigen::Matrix4f _pose, ros::Time _time, std::string _frameId);
Eigen::Matrix4f poseStamped2Eigen(geometry_msgs::PoseStamped _msg);

int main(int _argc, char **_argv) {

	if(_argc < 2){
		std::cout << "usage: rosrun hecatonquiros arm_position_holder_publisher_vicon /uav_1/mavros/local_position/pose /hecatonquiros/right_arm/out/pose" << std::endl;
		return -1;
	}

	ros::init(_argc, _argv, "arm_tester");

	ros::NodeHandle nh;


	Eigen::Matrix4f trasnformUavArms = Eigen::Matrix4f::Identity();
	trasnformUavArms(0,3) = 0;
	trasnformUavArms(1,3) = 0;
	trasnformUavArms(2,3) = 0.125;
	Eigen::Matrix3f rotUavArms;
	rotUavArms = 		Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())*
						Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())* 
						Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

	trasnformUavArms.block<3,3>(0,0) = rotUavArms;

	Eigen::Matrix4f poseUav = Eigen::Matrix4f::Identity(); 
	Eigen::Matrix4f relativePoseArms = Eigen::Matrix4f::Identity(); 

	ros::Time uavPoseTime;
	ros::Subscriber uavSubscriber = nh.subscribe<geometry_msgs::PoseStamped> (_argv[1],1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
		poseUav = poseStamped2Eigen(*_msg);
		uavPoseTime = _msg->header.stamp;
	});

	ros::Subscriber relativeArmPose = nh.subscribe<geometry_msgs::PoseStamped> (_argv[2],1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
		relativePoseArms = poseStamped2Eigen(*_msg);
	});

	ros::Publisher globalReferenePosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/arm_control_position/global_reference",1);
	ros::Publisher localReferenePosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/arm_control_position/local_reference",1);
	ros::Publisher targetPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("/hecatonquiros/right_arm/in/target_pose_line3d",1);
	
	ros::AsyncSpinner spinner(2);
	spinner.start();


	std::cout << "Press any key to get reference position" << std::endl;

	Eigen::Matrix4f globalReferencePose = relativePoseArms*trasnformUavArms*poseUav; 

	ros::Rate rate(30);

	while(ros::ok()){

		Eigen::Matrix4f targetArmPose;
		targetArmPose = trasnformUavArms*poseUav.inverse()*globalReferencePose;


		globalReferenePosePublisher.publish(eigen2PoseStamped(globalReferencePose, uavPoseTime, "map"));
		localReferenePosePublisher.publish(eigen2PoseStamped(targetArmPose, uavPoseTime, "map"));
		targetPosePublisher.publish(eigen2PoseStamped(targetArmPose, uavPoseTime, "map"));

		rate.sleep();
	}

}




geometry_msgs::PoseStamped eigen2PoseStamped(Eigen::Matrix4f _pose, ros::Time _time, std::string _frameId){
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = _frameId;
	msg.header.stamp = _time;
	msg.pose.position.x = _pose(0,3);
	msg.pose.position.y = _pose(1,3);
	msg.pose.position.z = _pose(2,3);
	Eigen::Quaternionf q(_pose.block<3,3>(0,0));
	msg.pose.orientation.x = q.x();
	msg.pose.orientation.y = q.y();
	msg.pose.orientation.z = q.z();
	msg.pose.orientation.w = q.w();
	return msg;
}

Eigen::Matrix4f poseStamped2Eigen(geometry_msgs::PoseStamped _msg){
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	pose(0,3) = _msg.pose.position.x;
	pose(1,3) = _msg.pose.position.y;
	pose(2,3) = _msg.pose.position.z;
	Eigen::Quaternionf q;
	q.x() = _msg.pose.orientation.x;
	q.y() = _msg.pose.orientation.y;
	q.z() = _msg.pose.orientation.z;
	q.w() = _msg.pose.orientation.w;
	pose.block<3,3>(0,0) = q.matrix();
	
	return pose;
}