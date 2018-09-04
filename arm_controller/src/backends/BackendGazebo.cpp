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

#include <hecatonquiros/backends/BackendGazebo.h>

#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>

#include <math.h>
#include <sstream>
#include <iostream>
#include <string>


namespace hecatonquiros{
//---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::init(const Config &_config){
		ros::NodeHandle n;
  		mtopic=_config.topic;	
		marmId=_config.armId;
 		joint_pub = n.advertise<sensor_msgs::JointState>(mtopic, 1000);
		return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::pose(const Eigen::Matrix4f &_pose, bool _blocking){
		return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::joints(std::vector<float> &_joints, bool _blocking){
	//while (ros::ok())
 	//{
		int cnt=0;
        sensor_msgs::JointState j_msg;

		if(marmId==2){
			j_msg.header.seq=cnt;
			j_msg.header.frame_id="0";
			j_msg.position.push_back (_joints[0]);
			j_msg.position.push_back (_joints[1]);
			j_msg.position.push_back (_joints[2]);   
        	            j_msg.name.push_back ("right/arm_0_bottom_joint");
        	            j_msg.name.push_back ("right/arm_1_joint");
        	            j_msg.name.push_back ("right/arm_2_joint");
			j_msg.header.stamp = ros::Time::now();					
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
		}

	    joint_pub.publish(j_msg);      
	    cnt++;
	    return true;
	  //}
		
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::claw(const int _action){
		return false;
    }

	//-----------------------------------------------------------------------------------------------------------------
    int BackendGazebo::jointPos(const int _id){
        return 0;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendGazebo::jointLoad(const int _id){
        return 0;
    }

	//-----------------------------------------------------------------------------------------------------------------
    int BackendGazebo::jointTorque(const int _id, const bool _enable){
        return 0;
    }

}
