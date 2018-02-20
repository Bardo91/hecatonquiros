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


#include <arm_controller/Arm4DoF.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cassert>
#include <ros/ros.h>

namespace hecatonquiros{
    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const Backend::Config &_config) {
        std::string mArmPrefix = "";
        ros::NodeHandle n;
        ros::Publisher mJointPublisher = n.advertise<sensor_msgs::JointState>(mArmPrefix+"/joint_states", 1000);
        std::cout << "Loading SRDF" <<std::endl;
        TiXmlDocument docSrdf("/home/bardo91/programming/catkin_positioner/src/PositionerEndTool/hecatonquiros/gazebo_simulation/config/arm4DoF.srdf");
        docSrdf.LoadFile();
        std::cout << "Loading URDF" <<std::endl;
        TiXmlDocument docUrdf("/home/bardo91/programming/catkin_positioner/src/PositionerEndTool/hecatonquiros/gazebo_simulation/urdf/arm4DoF.urdf");
        docUrdf.LoadFile();
        std::cout << "LOADED FILES" <<std::endl;
        rdf_loader::RDFLoader rdf_loader(&docUrdf, &docSrdf);
        srdf::ModelSharedPtr srdf = rdf_loader.getSRDF();
        urdf::ModelInterfaceSharedPtr urdf_model = rdf_loader.getURDF();

        std::cout <<srdf <<std::endl;
        std::cout <<urdf_model <<std::endl;

        robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

        robot_model_->printModelInfo(std::cout);
        
        joint_model_group_ = robot_model_->getJointModelGroup("arm4dof");

        const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();

        robot_state_.reset(new robot_state::RobotState(robot_model_));
        robot_state_->setToDefaultValues();

        int a;

        while(ros::ok()){
            std::vector<double> joint_values;
            robot_state_->setToRandomPositions(joint_model_group_);
            robot_state_->update ();
            robot_state_->updateLinkTransforms ();
            robot_state_->update ();

            robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
            for (std::size_t i = 0; i < jm_names.size(); ++i) {
                ROS_INFO("Joint %s: %f", jm_names[i].c_str(), joint_values[i]);
            }
            ROS_INFO_STREAM("Current state is " << (robot_state_->satisfiesBounds() ? "valid" : "not valid"));

            /* Enforce the joint limits for this state and check again*/
            robot_state_->enforceBounds();
            ROS_INFO_STREAM("Current state is " << (robot_state_->satisfiesBounds() ? "valid" : "not valid"));
            const Eigen::Affine3d &end_effector_state = robot_state_->getGlobalLinkTransform("arm_2");

            /* Print end-effector pose. Remember that this is in the model frame */
            ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
            ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

            sensor_msgs::JointState j_msg;

            int counter = 0;
            while(counter < 100){
                j_msg.header.stamp = ros::Time::now();		
                j_msg.header.seq=0;
                j_msg.header.frame_id=mArmPrefix;
                j_msg.position.push_back (joint_values[0]);
                j_msg.position.push_back (joint_values[1]);
                j_msg.position.push_back (joint_values[2]);   
                j_msg.name.push_back (mArmPrefix+"arm_0_bottom_joint");
                j_msg.name.push_back (mArmPrefix+"arm_1_joint");
                j_msg.name.push_back (mArmPrefix+"arm_2_joint");
                
                mJointPublisher.publish(j_msg);   
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                counter++;
            }
            bool found_ik = robot_state_->setFromIK(joint_model_group_, end_effector_state, 10, 0.1);
            // Now, we can print out the IK solution (if found):
            if (found_ik) {
                std::cout << "Inverse kinematic" << std::endl;
                robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
                for (std::size_t i = 0; i < jm_names.size(); ++i) {
                    ROS_INFO("Joint %s: %f", jm_names[i].c_str(), joint_values[i]);
                }
            } else {
                ROS_INFO("Did not find IK solution");
            }
            std::cout << "-------------------------------------" << std::endl;
        }

        //mBackend = Backend::create(_config);
        //mArmId = _config.armId;

        /*mRobotModelLoader = new robot_model_loader::RobotModelLoader("/robot_description");
        mKinematicModel  = mRobotModelLoader->getModel();
	    
        mKinematicState = robot_state::RobotStatePtr(new robot_state::RobotState(mKinematicModel));
        mKinematicState->setToDefaultValues();
        mJointsGroup = mKinematicModel->getJointModelGroup("arm4dof");
        mJointNames = mJointsGroup->getVariableNames();

	    mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);

        for(auto &joint: mArmJoints) std::cout << joint << ", ";
        std::cout << std::endl;

        mKinematicState->setToRandomPositions(mJointsGroup);
	    mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
        for(auto &joint: mArmJoints) std::cout << joint << ", ";
        std::cout << std::endl;
        mKinematicState->enforceBounds();
	    mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
        Eigen::Affine3d end_effector_state = mKinematicState->getGlobalLinkTransform("arm_2");
    
        ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
        ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());*/        
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::home(){
        joints({mHome1, mHome2, mHome3, mHome4, mHome5});
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::joints(std::vector<double> _q) {
        /*assert(_q.size() >= mArmJoints.size());

        for(unsigned i = 0; i < mArmJoints.size(); i++){
            mArmJoints[i] = _q[i];
        }
        mKinematicState->setJointGroupPositions(mJointsGroup, mArmJoints);
        if(mBackend != nullptr){
            mBackend->joints(std::vector<float>(mArmJoints.begin(), mArmJoints.end()));
        }*/
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<double> Arm4DoF::joints() {
        /*mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
        return mArmJoints;*/
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::position(Eigen::Matrix4f _position, float _wirst) {
        if(checkIk(_position)){
            joints(mArmJoints); // send joints to arduino
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f Arm4DoF::position() {
        //Eigen::Affine3d endState = mKinematicState->getGlobalLinkTransform("arm_2");
        //std::cout << endState.matrix() <<std::endl;
        return Eigen::Matrix4f::Identity();//endState.cast<float>().matrix();
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position){
        std::vector<Eigen::Matrix4f> ts;
        std::vector<double> angles;
        return checkIk(_position, angles, ts);
    }


    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position, std::vector<Eigen::Matrix4f> &_transformations){
        std::vector<double> angles;
        return checkIk(_position, angles,_transformations);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position, std::vector<double> &_angles){
        std::vector<Eigen::Matrix4f> ts;
        return checkIk(_position,_angles, ts);
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _position, std::vector<double> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        /*Eigen::Affine3f affine3f(_position);
        bool foundIk = mKinematicState->setFromIK(mJointsGroup, affine3f.cast<double>() , 10, 0.1);
        if(foundIk){
            mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
            _angles = mArmJoints;
            return true;
        }else{
            return false;
        }
*/
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::directKinematic(const std::vector<double> &_angles, std::vector<Eigen::Matrix4f> &_transformations){
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::lastTransformations(Eigen::Matrix4f &_t0, Eigen::Matrix4f &_t1, Eigen::Matrix4f &_t2) {
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::closeClaw(){
        if(mBackend != nullptr){
            mBackend->claw(0);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::openClaw(){
        if(mBackend != nullptr){
            mBackend->claw(2);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::stopClaw(){
        if(mBackend != nullptr){
            mBackend->claw(1);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::isInit() const{
        return mBackend != nullptr;
    }
}