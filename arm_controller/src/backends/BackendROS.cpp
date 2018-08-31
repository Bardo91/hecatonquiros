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
		mJointsReq 	= nh.serviceClient<hecatonquiros::ReqData>("/backendROS/arm_"+std::to_string(mArmId)+"/joints_req");
		mJointIDReq = nh.serviceClient<hecatonquiros::ReqData>("/backendROS/arm_"+std::to_string(mArmId)+"/jointId_req");
		mLoadIDReq 	= nh.serviceClient<hecatonquiros::ReqData>("/backendROS/arm_"+std::to_string(mArmId)+"/loadId_req");


		mJointsSubscriber = nh.subscribe<sensor_msgs::JointState>("/backendROS/arm_"+std::to_string(mArmId)+"/joints_sub", 1, \
        [this](const sensor_msgs::JointStateConstPtr& _msg) {
            mJoints.resize(_msg->position.size());
			for(int i = 0; i < _msg->position.size(); i++){
				mGuard.lock();
				mJoints[i] = _msg->position[i];
				mGuard.unlock();
			}
    	});

		mJointIDSubscriber = nh.subscribe<std_msgs::Int32>("/backendROS/arm_"+std::to_string(mArmId)+"/jointId_sub", 1, \
		[this](const std_msgs::Int32::ConstPtr& _msg){
			mGuard.lock();
			mJoint = _msg->data;
			mGuard.unlock();
		});
		mLoadIDSubscriber = nh.subscribe<std_msgs::Int32>("/backendROS/arm_"+std::to_string(mArmId)+"/loadId_sub", 1, \
		[this](const std_msgs::Int32::ConstPtr& _msg){
			mGuard.lock();
			mLoad = _msg->data;
			mGuard.unlock();
		});

		return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendROS::pose(const Eigen::Matrix4f &_pose, bool _blocking){
		return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendROS::joints(std::vector<float> &_joints, bool _blocking){
		
        sensor_msgs::JointState jointsMsg;
	    jointsMsg.header.stamp = ros::Time::now();
		for(auto&j:_joints){
            jointsMsg.position.push_back(j);
        }				
		
	    mJointsPublisher.publish(jointsMsg);   

	    return true;
		
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool BackendROS::claw(const int _action){

		hecatonquiros::ReqData srv;
		srv.request.req = true;
		srv.request.id = _action;
		if(mClawReq.call(srv)){
			if(srv.response.success){
				return true;
			}
		}else{
			std::cout << "BCROS: Failed to call service of claw" << std::endl;
			return false;
		}
    }

	//-----------------------------------------------------------------------------------------------------------------
    int BackendROS::jointPos(const int _id){
		
		hecatonquiros::ReqData srv;
		srv.request.req = true;
		srv.request.id = _id;
		if(mJointIDReq.call(srv)){
			if(srv.response.success){
				return mJoint;
			}
		}else{
			std::cout << "BCROS: Failed to call service of jointPos" << std::endl;
			return -1;
		}
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendROS::jointLoad(const int _id){

        hecatonquiros::ReqData srv;
		srv.request.req = true;
		srv.request.id = _id;
		if(mLoadIDReq.call(srv)){
			if(srv.response.success){
				return mLoad;
			}
		}else{
			std::cout << "BCROS: Failed to call service of jointLoad" << std::endl;
			return -1;
		}
    }

	//-----------------------------------------------------------------------------------------------------------------
    std::vector<float> BackendROS::joints(int nJoints){

		hecatonquiros::ReqData srv;
		srv.request.req = true;
		srv.request.id = nJoints;
		if(mJointsReq.call(srv)){
			if(srv.response.success){
				return mJoints;
			}else{
				std::cout << "BCROS: Response call service Joints is FALSE" << std::endl;
				std::vector<float> dummy;
				dummy.resize(nJoints);
				for(int i = 0; i < nJoints; i++){
					dummy[i] = 0;
				}
				return dummy;
			}
		}else{
			std::cout << "BCROS: Failed to call service of joints" << std::endl;
			std::vector<float> dummy;
			dummy.resize(nJoints);
			for(int i = 0; i < nJoints; i++){
				dummy[i] = 0;
			}
			return dummy;
		}
    }

}
