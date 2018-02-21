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
	
	int left_cnt=1;
	int right_cnt=1;
 	int joint;
  	
  	
	
	ros::NodeHandle n;
  	mleft_topic=_config.left_topic;
	mright_topic=_config.right_topic;
	//mGarmId=_config.GarmId;
 	left_joint_pub = n.advertise<sensor_msgs::JointState>(mleft_topic, 1000);
	right_joint_pub = n.advertise<sensor_msgs::JointState>(mright_topic, 1000);
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
    	
    	  	int left_cnt=0;
		int right_cnt=0;
                sensor_msgs::JointState left_j_msg;
		sensor_msgs::JointState right_j_msg;


			left_j_msg.header.seq=left_cnt;
                        left_j_msg.header.frame_id="0";
                        left_j_msg.position.push_back (_joints[0]);
                        left_j_msg.position.push_back (_joints[1]);
                        left_j_msg.position.push_back (_joints[2]);
                        left_j_msg.name.push_back ("left/arm_0_bottom_joint");
                        left_j_msg.name.push_back ("left/arm_1_joint");
                        left_j_msg.name.push_back ("left/arm_2_joint");
                        left_j_msg.header.stamp = ros::Time::now();

			right_j_msg.header.seq=right_cnt;
                        right_j_msg.header.frame_id="0";
                        right_j_msg.position.push_back (_joints[0]);
                        right_j_msg.position.push_back (_joints[1]);
                        right_j_msg.position.push_back (_joints[2]);
                        right_j_msg.name.push_back ("right/arm_0_bottom_joint");
                        right_j_msg.name.push_back ("right/arm_1_joint");
                        right_j_msg.name.push_back ("right/arm_2_joint");
                        right_j_msg.header.stamp = ros::Time::now();
			/*
                        if(mGarmId==0){
                        j_msg.header.seq=cnt;
                        j_msg.header.frame_id="0";
                        j_msg.position.push_back (_joints[0]);
                        j_msg.position.push_back (_joints[1]);
                        j_msg.position.push_back (_joints[2]);
                        j_msg.name.push_back ("arm_0_bottom_joint");
                        j_msg.name.push_back ("arm_1_joint");
                        j_msg.name.push_back ("arm_2_joint");
                        j_msg.header.stamp = ros::Time::now();
                        ROS_INFO("[Talker]:Publishing alone");
                        }
                        else if(mGarmId==1){
			j_msg.header.seq=cnt;
			j_msg.header.frame_id="0";
			j_msg.position.push_back (_joints[0]);
			j_msg.position.push_back (_joints[1]);
			j_msg.position.push_back (_joints[2]);   
                        j_msg.name.push_back ("right/arm_0_bottom_joint");
                        j_msg.name.push_back ("right/arm_1_joint");
                        j_msg.name.push_back ("right/arm_2_joint");
			j_msg.header.stamp = ros::Time::now();		
			ROS_INFO("[Talker]:Publishing right arm");
			}
			else{
			j_msg.header.seq=cnt;
			j_msg.header.frame_id="0";
			j_msg.position.push_back (_joints[0]);
			j_msg.position.push_back (_joints[1]);
			j_msg.position.push_back (_joints[2]);   
                        j_msg.name.push_back ("left/arm_0_bottom_joint");
                        j_msg.name.push_back ("left/arm_1_joint");
                        j_msg.name.push_back ("left/arm_2_joint");
			j_msg.header.stamp = ros::Time::now();		
			ROS_INFO("[Talker]:Publishing left arm");
			}*/

		
	

	
	    left_joint_pub.publish(left_j_msg);  
	    right_joint_pub.publish(right_j_msg); 
	    left_cnt++;
	    right_cnt++;
	    return true;
	  //}
		

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::claw(const int _action){
	return false;

    }

    

}
