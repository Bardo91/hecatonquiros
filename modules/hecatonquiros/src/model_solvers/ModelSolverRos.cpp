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

#include <hecatonquiros/model_solvers/ModelSolverRos.h>
#include <hecatonquiros/SetPose.h>
#include <hecatonquiros/SetJoints.h>


namespace hecatonquiros{

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverRos::init(const ModelSolver::Config &_config){
        #ifdef HAS_ROS
            if(!ros::isInitialized()){
                std::cout << "ERROR, ROS should be initialized out of the ModelSolver" << std::endl;
                return false;
            }else{
                mRobotName = _config.robotName;
                ros::NodeHandle nh;
                mPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>(mRobotName + "/joints", 1, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
                        mLastPose = *_msg;
                    });
                mJointsSubscriber = nh.subscribe<sensor_msgs::JointState>(mRobotName + "/pose", 1, [&](const sensor_msgs::JointState::ConstPtr &_msg){
                        mLastJoints = *_msg;
                    });
                mTransformsSubscriber = nh.subscribe<geometry_msgs::PoseArray>(mRobotName + "/joints_transform", 1, [&](const geometry_msgs::PoseArray::ConstPtr &_msg){
                        mLastJointTransforms = *_msg;
                    });
            }
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return false;    
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverRos::joints(std::vector<float> &_joints){
        #ifdef HAS_ROS
            ros::NodeHandle n;
            hecatonquiros::SetJoints jointsSrv;
            for(auto &j: _joints){
                jointsSrv.request.inJoints.position.push_back(j);
            }
            ros::ServiceClient client = n.serviceClient<hecatonquiros::SetJoints>(mRobotName + "/set_joints");
            client.call(jointsSrv);
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return false;    
        #endif         
    }

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<float> ModelSolverRos::joints() const{
        #ifdef HAS_ROS
            std::vector<float> joints;
            sensor_msgs::JointState lastJoints = mLastJoints;
            for(auto &j:lastJoints.position){
                joints.push_back(j);
            }
            return joints;
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return std::vector<float>();    
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverRos::jointsTransform(std::vector<Eigen::Matrix4f> &_transforms){
        #ifdef HAS_ROS
            geometry_msgs::PoseArray transforms = mLastJointTransforms;

            for(auto &transform: transforms.poses){
                Eigen::Quaternionf q(transform.orientation.w, transform.orientation.x, transform.orientation.y, transform.orientation.z);
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                pose.block<3,3>(0,0) = q.matrix();
                pose(0,3) = transform.position.x;
                pose(1,3) = transform.position.y;
                pose(2,3) = transform.position.z;
                _transforms.push_back(pose);
            }
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return ;    
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f ModelSolverRos::jointTransform(int _idx){
        #ifdef HAS_ROS
            geometry_msgs::Pose transform = mLastJointTransforms.poses[_idx];
            
            Eigen::Quaternionf q(transform.orientation.w, transform.orientation.x, transform.orientation.y, transform.orientation.z);
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block<3,3>(0,0) = q.matrix();
            pose(0,3) = transform.position.x;
            pose(1,3) = transform.position.y;
            pose(2,3) = transform.position.z;

            return pose;
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return Eigen::Matrix4f::Identity();    
        #endif 
        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverRos::checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, IK_TYPE _type){
        #ifdef HAS_ROS
            ros::NodeHandle n;
            hecatonquiros::SetPose poseSrv;
            poseSrv.request.inPose.pose.position.x = _pose(0,3);
            poseSrv.request.inPose.pose.position.y = _pose(1,3);
            poseSrv.request.inPose.pose.position.z = _pose(2,3);

            Eigen::Quaternionf q(_pose.block<3,3>(0,0));

            poseSrv.request.inPose.pose.orientation.x = q.x();
            poseSrv.request.inPose.pose.orientation.y = q.y();
            poseSrv.request.inPose.pose.orientation.z = q.z();
            poseSrv.request.inPose.pose.orientation.w = q.w();
            if(_type == IK_TYPE::IK_3D)
                poseSrv.request.forceOri = 0;
            else if(_type == IK_TYPE::IK_5D)
                poseSrv.request.forceOri = 1;
            else if(_type == IK_TYPE::IK_6D)
                poseSrv.request.forceOri = 2;
            else
                poseSrv.request.forceOri = 0;
                
            poseSrv.request.single = true;
            poseSrv.request.set = true;

            ros::ServiceClient client = n.serviceClient<hecatonquiros::SetPose>(mRobotName + "/set_pose");
            if(client.call(poseSrv)){
                if(poseSrv.response.outJoints.size() > 0){
                    for(auto &j:poseSrv.response.outJoints[0].position){
                        _joints.push_back(j);
                    }
                    return true;
                }else{
                    return false;
                }
            }else{
                return false;
            }
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return false;    
        #endif 
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverRos::checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, IK_TYPE _type){
        #ifdef HAS_ROS
            ros::NodeHandle n;
            hecatonquiros::SetPose poseSrv;
            poseSrv.request.inPose.pose.position.x = _pose(0,3);
            poseSrv.request.inPose.pose.position.y = _pose(1,3);
            poseSrv.request.inPose.pose.position.z = _pose(2,3);

            Eigen::Quaternionf q(_pose.block<3,3>(0,0));

            poseSrv.request.inPose.pose.orientation.x = q.x();
            poseSrv.request.inPose.pose.orientation.y = q.y();
            poseSrv.request.inPose.pose.orientation.z = q.z();
            poseSrv.request.inPose.pose.orientation.w = q.w();

            if(_type == IK_TYPE::IK_3D)
                poseSrv.request.forceOri = 0;
            else if(_type == IK_TYPE::IK_5D)
                poseSrv.request.forceOri = 1;
            else if(_type == IK_TYPE::IK_6D)
                poseSrv.request.forceOri = 2;
            else
                poseSrv.request.forceOri = 0;

            poseSrv.request.single = false;
            poseSrv.request.set = true;

            std::cout << _pose << std::endl;

            ros::ServiceClient client = n.serviceClient<hecatonquiros::SetPose>(mRobotName + "/set_pose");
            if(client.call(poseSrv)){
                for(unsigned i = 0; i <  poseSrv.response.outJoints.size(); i++){
                    _joints.push_back(std::vector<float>());
                    for(auto &j:poseSrv.response.outJoints[i].position){
                        _joints.back().push_back(j);
                    }
                }
                return true;
            }else{
                return false;
            }
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return false;    
        #endif 
    }

    Eigen::Matrix4f ModelSolverRos::testIk(const std::vector<float> &_joints){
        #ifdef HAS_ROS
            std::cout << "ModelSolverRos does not implement testIK method yet" << std::endl;
            return Eigen::Matrix4f::Identity();    
        #else
            std::cout << "ROS not installed, cannot use ModelSolverRos" << std::endl;
            return Eigen::Matrix4f::Identity();    
        #endif 
    }
}
