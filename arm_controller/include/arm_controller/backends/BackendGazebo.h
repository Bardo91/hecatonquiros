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


#ifndef HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKENDGAZEBO_H_
#define HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKENDGAZEBO_H_

#include <arm_controller/backends/Backend.h>
#include "ros/ros.h"
namespace hecatonquiros{
    class BackendGazebo: public Backend{
    public:
        /// Default constructor
        BackendGazebo():Backend(){}

        /// This method is not implemented in arduino backend, it sends false by default.
        virtual bool pose(const Eigen::Matrix4f &_pose, bool _blocking = false);

        /// This method move the joints of the arm to the desired angle.
        /// \param _joints: vector containing the joints
        /// \param _blocking: set blocking or not blocking operation
        /// \return true if joints are send or set without errors, false if something failed.
        virtual bool joints(const std::vector<float> &_joints, bool _blocking = false);

        /// Method for actuating to claws if implemented and attached
        /// \param _action: 0 close, 1 stop, 2 open;
        virtual bool claw(const int _action);
    private:
        virtual bool init(const Config &_config);

	std::string mleft_topic;
	std::string mright_topic;
	ros::Publisher left_joint_pub; 
	ros::Publisher right_joint_pub; 
	int mGarmId;
    };
}

#endif
