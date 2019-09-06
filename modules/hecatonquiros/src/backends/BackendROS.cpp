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

#include <hecatonquiros/backends/BackendROS.h>

namespace hecatonquiros{
//---------------------------------------------------------------------------------------------------------------------
    bool BackendROS::init(const Config &_config){

		ros::NodeHandle nh;
		mArmId = _config.armId;

 		mJointsPublisher = nh.advertise<sensor_msgs::JointState>("/backendROS/arm_"+std::to_string(mArmId)+"/joints", 1);
		
		mClawReq 	= nh.serviceClient<hecatonquiros::ReqData>("/backendROS/arm_"+std::to_string(mArmId)+"/claw_req");
		mConfigReq 	= nh.serviceClient<hecatonquiros::ConfigData>("/backendROS/config_req");

		mJointsSubscriber = nh.subscribe<sensor_msgs::JointState>("/backendROS/arm_"+std::to_string(mArmId)+"/joints_sub", 1, \
        [this](const sensor_msgs::JointStateConstPtr& _msg) {
            mJoints.resize(_msg->position.size());
			for(int i = 0; i < _msg->position.size(); i++){
				mGuard.lock();
				mJoints[i] = _msg->position[i];
				mGuard.unlock();
			}
    	});

		hecatonquiros::ConfigData srv;
		srv.request.req = true;
		srv.request.id = mArmId;
		srv.request.ndof =  _config.ndof; 
		srv.request.serialport = _config.port;
		srv.request.configxml = _config.configXML;
		if(mConfigReq.call(srv)){
			if(srv.response.success){
				std::cout << "BCROS: Service of Config Arms success" << std::endl;
				return true;
			}else{
				return false;
			}
		}else{
			std::cout << "BCROS: Failed to call service of Config Arms" << std::endl;
			return false;
		}		

		return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendROS::joints(std::vector<float> &_joints, bool _blocking){
		
        sensor_msgs::JointState jointsMsg;
	    jointsMsg.header.stamp = ros::Time::now();
		jointsMsg.effort.push_back(mArmId);
		for(auto&j:_joints){
            jointsMsg.position.push_back(j);
        }				
		
	    mJointsPublisher.publish(jointsMsg);   

	    return true;
		
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendROS::claw(const int _action, bool _blocking){

		hecatonquiros::ReqData srv;
		srv.request.req = true;
		srv.request.id = mArmId;
		srv.request.data = _action;
		if(mClawReq.call(srv)){
			if(srv.response.success){
				return true;
			}else{
				return false;
			}
		}else{
			std::cout << "BCROS: Failed to call service of claw" << std::endl;
			return false;
		}
    }

	//-----------------------------------------------------------------------------------------------------------------
    std::vector<float> BackendROS::joints(int nJoints, bool _blocking){
		return mJoints;
    }

	//---------------------------------------------------------------------------------------------------------------------
    bool BackendROS::pose(const Eigen::Matrix4f &_pose, bool _blocking){
		return false;
    }

	//---------------------------------------------------------------------------------------------------------------------
    int BackendROS::jointPos(const int _id){
		return -1;
    }

	//---------------------------------------------------------------------------------------------------------------------
    int BackendROS::jointLoad(const int _id){
		return -1;
    }

	//---------------------------------------------------------------------------------------------------------------------
    int BackendROS::jointTorque(const int _id, const bool _enable){
		return -1;
    }

}
