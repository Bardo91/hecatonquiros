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
		if(!ros::isInitialized()){
			ros::NodeHandle n;
			mArmPrefix = _config.armPrefix;
			mArmId = _config.armId;
			mJointPublisher = n.advertise<sensor_msgs::JointState>(mArmPrefix, 1000);
			return true;
		}else{
			return false;
		}
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::pose(const Eigen::Matrix4f &_pose, bool _blocking){
		return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::joints(const std::vector<float> &_joints, bool _blocking){
		sensor_msgs::JointState j_msg;

		j_msg.header.stamp = ros::Time::now();		
		j_msg.header.seq=0;
		j_msg.header.frame_id=mArmPrefix;
		j_msg.position.push_back (_joints[0]);
		j_msg.position.push_back (_joints[1]);
		j_msg.position.push_back (_joints[2]);   
		j_msg.name.push_back (mArmPrefix+"/arm_0_bottom_joint");
		j_msg.name.push_back (mArmPrefix+"/arm_1_joint");
		j_msg.name.push_back (mArmPrefix+"/arm_2_joint");
		
		mJointPublisher.publish(j_msg);   
		return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::claw(const int _action){
	return false;

    }

    

}
