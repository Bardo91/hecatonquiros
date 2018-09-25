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


#ifndef HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKENDROS_H_
#define HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKENDROS_H_

#include <hecatonquiros/backends/Backend.h>
#include <ros/ros.h>
#include <mutex>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <thread>

#include "hecatonquiros/ReqData.h"
#include "hecatonquiros/ConfigData.h"

namespace hecatonquiros{
    class BackendROS: public Backend{
    public:
        /// Default constructor
        BackendROS():Backend(){}

        /// This method is not implemented in arduino backend, it sends false by default.
        virtual bool pose(const Eigen::Matrix4f &_pose, bool _blocking = false);

        /// This method move the joints of the arm to the desired angle.
        /// \param _joints: vector containing the joints
        /// \param _blocking: set blocking or not blocking operation
        /// \return true if joints are send or set without errors, false if something failed.
        virtual bool joints(std::vector<float> &_joints, bool _blocking = false);

        /// Method for actuating to claws if implemented and attached
        /// \param _action: 0 close, 1 stop, 2 open;
        virtual bool claw(const int _action, bool _blocking = false);

        /// \brief abstract method for read position of a servo
        /// \param _id: id joint to read
        virtual int jointPos(const int _id);

        /// \brief abstract method for read Load of a servo
        /// \param _id: id joint to read
        virtual int jointLoad(const int _id);

        /// \brief abstract method for enable/disable servo torque
        /// \param _id
        /// \param _enable
        virtual int jointTorque(const int _id, const bool _enable);
  
        virtual std::vector<float> joints(int nJoints);
    private:
        virtual bool init(const Config &_config);

    private:
        std::mutex mGuard;
        
        ros::Publisher mJointsPublisher;
        ros::ServiceClient  mClawReq, mConfigReq;
        ros::Subscriber mJointsSubscriber;

        std::vector<float> mJoints;
	    int mArmId;
    };
}

#endif
