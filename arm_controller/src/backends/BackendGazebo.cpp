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
#include <iomanip>
#include <math.h>
#include <sstream>
#include <ros/subscribe_options.h>
#include <iostream>
#include <string>
#include <fstream>


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
    bool BackendGazebo::joints(const std::vector<float> &_joints, bool _blocking){
	
    	
    	  	
		int cnt=0;
        sensor_msgs::JointState j_msg;
		int len_fixed_string = 0;
		int next_iteration=0;
		int i;
		int cnt_vector=0;
		std::ifstream inFile;
		std::string string;
		std::string var_string;
		std::string item_name;
		std::ifstream nameFileout;
		std::string string_vector[10];

		//Detect the names

		nameFileout.open("/home/luis/catkin_ws_2/src/hecatonquiros/arm_description/urdf/4dof.urdf");
		while (nameFileout >> item_name)
		{
  			  //std::cout << item_name << "\n";
	
			if(next_iteration==2)
				{
					len_fixed_string=item_name.length();
					if(len_fixed_string==13)   //13 is the amount of characters in type="fixed">
					{
					//Deleting
					string_vector[cnt_vector]="deleted";
					}
					else
					{
					//Saving because the joint is continuous
					cnt_vector++;
					}

				next_iteration=0;
				}


			if(next_iteration==1)
				{
				string=item_name;	
				var_string.resize(string.length()-7); //7 is the number of characters that I dont need in the xml
				for(i=0;i<=(string.length()-7) ;i++)
				{
				var_string[i]=string[i+6];
				}
				string_vector[cnt_vector]=var_string;
				next_iteration++;
				}
			
			
			if(item_name=="<joint") //Search the joints
				{
					next_iteration++;
				}
			

		}
		for(i=0;i<cnt_vector;i++)
		{
		std::cout << "\n" << string_vector[i];
		}
		nameFileout.close();






		//Send messages

			if(marmId==2){
			j_msg.header.seq=cnt;
			j_msg.header.frame_id="0";
			j_msg.position.push_back (_joints[0]);
			j_msg.position.push_back (_joints[1]);
			j_msg.position.push_back (_joints[2]);   
                        j_msg.name.push_back ("right/arm_0_bottom_joint");
                        j_msg.name.push_back ("right/arm_1_joint");
                        j_msg.name.push_back ("right/arm_2_joint");

						//j_msg.name.push_back (string_vector[3]);
                        //j_msg.name.push_back (string_vector[4]);
                        //j_msg.name.push_back (string_vector[5]);
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
						//j_msg.name.push_back (string_vector[0]);
                        //j_msg.name.push_back (string_vector[1]);
                        //j_msg.name.push_back (string_vector[2]);
			j_msg.header.stamp = ros::Time::now();					
			}

		
	

	
	    joint_pub.publish(j_msg);      
	    cnt++;
	    return true;
	  		

    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendGazebo::claw(const int _action){
	return false;

    }

    

}
