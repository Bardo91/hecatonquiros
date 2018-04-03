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


#ifndef HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVERROS_H_
#define HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVERROS_H_

#include <hecatonquiros/model_solvers/ModelSolver.h>

#include <thread>

#ifdef HAS_ROS
    #include <ros/ros.h>
    #include <geometry_msgs/PoseArray.h>
    #include <sensor_msgs/JointState.h>
    #include <geometry_msgs/PoseStamped.h>
#endif 
namespace hecatonquiros{
    class ModelSolverRos: public ModelSolver{
    public:
        /// Set joints of robot
        /// \param _joints: desired joints
        virtual void joints(const std::vector<float> &_joints);
        
        /// Get current joints of robot
        virtual std::vector<float> joints() const;
        
        /// Get transforms of joints
        virtual void jointsTransform(std::vector<Eigen::Matrix4f> &_transforms);

        /// Get transforms of specific joint
        virtual Eigen::Matrix4f jointTransform(int _idx);

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, IK_TYPE _type = IK_TYPE::IK_3D);

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: list of possible solutions joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, IK_TYPE _type = IK_TYPE::IK_3D);

        /// Five end effector pose given joints without moving the arm
        /// \param _joints: list of possible solutions joints for given pose
        virtual Eigen::Matrix4f testIk(const std::vector<float> &_joints);
    protected:
        virtual bool init(const ModelSolver::Config &_config);

    private:
        #ifdef HAS_ROS
            ros::Subscriber mPoseSubscriber, mJointsSubscriber, mTransformsSubscriber;
            geometry_msgs::PoseStamped mLastPose;
            sensor_msgs::JointState mLastJoints;
            geometry_msgs::PoseArray mLastJointTransforms;
        #endif 

        std::string mRobotName;

    };

}

#endif
