//---------------------------------------------------------------------------------------------------------------------
//  HECATONQUIROS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
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

#include <arm_controller/backends/BackendGazebo.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <sstream>
#include <ros/subscribe_options.h>
#include <iostream>
#include <string>


namespace hecatonquiros{
//---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::init(const Config &_config){
	
	int cnt=1;
 	int joint;
  	float pos_rad_0,pos_rad_1,pos_rad_2;
  	
	
	ros::NodeHandle n;
  	mtopic=_config.topic;
 	joint_pub = n.advertise<sensor_msgs::JointState>(mtopic, 1000);
	return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::pose(const Eigen::Matrix4f &_pose, bool _blocking){
	return false;

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::joints(const std::vector<float> &_joints, bool _blocking){
	//while (ros::ok())
 	 //{
    	
    	  	int cnt=0;
	 	sensor_msgs::JointState j_msg; 
		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "Introduce vector of radians: " ;
		//msg.data = ss.str();
		//ROS_INFO("[Talker]:%s ", msg.data.c_str());
		//std::cin>>pos_rad_0>>pos_rad_1>>pos_rad_2;
	
	

			j_msg.header.seq=cnt;
			j_msg.header.frame_id="0";
			j_msg.position.push_back (_joints[0]);
			j_msg.position.push_back (_joints[1]);
			j_msg.position.push_back (_joints[2]);   
			j_msg.name.push_back ("arm_0_bottom_joint");
			j_msg.name.push_back ("arm_1_joint");
			j_msg.name.push_back ("arm_2_joint");
			j_msg.header.stamp = ros::Time::now();		
			ROS_INFO("[Talker]:Publishing");
	

	
	    joint_pub.publish(j_msg);   
	    cnt++;
	    return true;
	  //}
		

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::claw(const int _action){
	return false;

    }

    

}
