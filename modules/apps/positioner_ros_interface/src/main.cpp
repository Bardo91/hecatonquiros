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


#include <hecatonquiros/Positioner.h>
#include <iostream>
#include <chrono>
#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>

geometry_msgs::PoseStamped eigen2PoseStamped(Eigen::Matrix4f _pose, ros::Time _time = ros::Time::now(), std::string  _frameId = "base_docking");
tf::Transform eigen2tf(Eigen::Matrix4f _pose, ros::Time _time = ros::Time::now());
Eigen::Matrix4f poseStamped2Eigen(geometry_msgs::PoseStamped _msg);

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "positioner_interface");

	hecatonquiros::Positioner dockingTool("/dev/ttyACM0", 115200);
	std::this_thread::sleep_for(std::chrono::seconds(2));

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle n;
	ros::ServiceServer lockerService = n.advertiseService<std_srvs::Trigger::Request,std_srvs::Trigger::Response>("/hecatonquiros/positioner/toggle_lock", 
		[&](std_srvs::Trigger::Request &_req, std_srvs::Trigger::Response &_res)->bool{
			dockingTool.toggleLock();
			_res.success = true;
			return true;
		}
	);

	ros::Publisher pubImu 			= n.advertise<sensor_msgs::Imu>				("/hecatonquiros/positioner/base_accel", 	1);
	ros::Publisher pubJoints 		= n.advertise<sensor_msgs::JointState>		("/hecatonquiros/positioner/joints", 		1);
	ros::Publisher pubBaseToHand 	= n.advertise<geometry_msgs::PoseStamped>	("/hecatonquiros/positioner/base_to_hand", 	1);
	ros::Publisher pubHandToBase 	= n.advertise<geometry_msgs::PoseStamped>	("/hecatonquiros/positioner/hand_to_base", 	1);

	tf::TransformBroadcaster br;

	ros::Rate publishRate(30);
	while(ros::ok()){
		auto angles = dockingTool.angles();
		auto accel = dockingTool.accelerationVector();
		Eigen::Matrix4f baseToHand, handToBase;
		dockingTool.baseToHand(baseToHand);
		dockingTool.handToBase(handToBase);

		auto timeNow = ros::Time::now();
		sensor_msgs::JointState jointData;
		sensor_msgs::Imu imuData;

		jointData.header.stamp = timeNow;
		jointData.header.frame_id = "base_docking";
		jointData.position.insert(jointData.position.end(), angles.begin(), angles.end());

		imuData.header.stamp = timeNow;
		imuData.header.frame_id = "base_docking";
		imuData.linear_acceleration.x = accel[0];
		imuData.linear_acceleration.y = accel[1];
		imuData.linear_acceleration.z = accel[2];

		pubImu.publish(imuData);
		pubJoints.publish(jointData);

		pubBaseToHand.publish(eigen2PoseStamped(baseToHand, timeNow, "base_docking"));
		pubHandToBase.publish(eigen2PoseStamped(handToBase, timeNow, "end_effector"));


		Eigen::Matrix4f t01,t12,t23,t34,t4f;
		dockingTool.lastTransforms(t01,t12,t23,t34,t4f);

		br.sendTransform(tf::StampedTransform(eigen2tf(t01), timeNow, "base_docking", "dt_t01"));
		br.sendTransform(tf::StampedTransform(eigen2tf(t12), timeNow, "dt_t01", "dt_t12"));
		br.sendTransform(tf::StampedTransform(eigen2tf(t23), timeNow, "dt_t12", "dt_t23"));
		br.sendTransform(tf::StampedTransform(eigen2tf(t34), timeNow, "dt_t23", "dt_t34"));
		br.sendTransform(tf::StampedTransform(eigen2tf(t4f), timeNow, "dt_t34", "dt_t4f"));

		publishRate.sleep();
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


tf::Transform eigen2tf(Eigen::Matrix4f _pose, ros::Time _time){
	Eigen::Quaternionf q(_pose.block<3,3>(0,0));
	
	tf::Transform transform;
	transform.setOrigin( 
		tf::Vector3(_pose(0,3), _pose(1,3), _pose(2,3) ) 
		);
	transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

	return transform;
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