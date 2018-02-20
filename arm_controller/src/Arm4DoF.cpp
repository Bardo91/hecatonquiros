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
#include <ros/package.h>

namespace hecatonquiros{
    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const Backend::Config &_config) {
        std::string mArmPrefix = "";
        ros::NodeHandle n;
        ros::Publisher mJointPublisher = n.advertise<sensor_msgs::JointState>(mArmPrefix+"/joint_states", 1000);

        robot_model_loader::RobotModelLoader robot_model_loader("/robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

        // Using the :moveit_core:`RobotModel`, we can construct a
        // :moveit_core:`RobotState` that maintains the configuration
        // of the robot. We will set all joints in the state to their
        // default values. We can then get a
        // :moveit_core:`JointModelGroup`, which represents the robot
        // model for a particular group, e.g. the "right_arm" of the PR2
        // robot.
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();
        const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm4dof");

        const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

        while(ros::ok()){
            // Get Joint Values
            // ^^^^^^^^^^^^^^^^
            // We can retreive the current set of joint values stored in the state for the right arm.
            std::vector<double> joint_values;
            // Joint Limits
            // ^^^^^^^^^^^^
            // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
            /* Set one joint in the right arm outside its joint limit */
            kinematic_state->setToRandomPositions(joint_model_group);
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

            /* Enforce the joint limits for this state and check again*/
            kinematic_state->enforceBounds();
            ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("arm_2");

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
            
            // Forward Kinematics
            // ^^^^^^^^^^^^^^^^^^
            // Now, we can compute forward kinematics for a set of random joint
            // values. Note that we would like to find the pose of the
            // "r_wrist_roll_link" which is the most distal link in the
            
            // Inverse Kinematics
            // ^^^^^^^^^^^^^^^^^^
            // We can now solve inverse kinematics (IK) for the right arm of the
            // PR2 robot. To solve IK, we will need the following:
            // * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain):
            // end_effector_state that we computed in the step above.
            // * The number of attempts to be made at solving IK: 5
            // * The timeout for each attempt: 0.1 s
            bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
            // Now, we can print out the IK solution (if found):
            if (found_ik)
            {
                std::cout << "Inverse kinematic" << std::endl;
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            }
            else
            {
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
        assert(_q.size() >= mArmJoints.size());

        for(unsigned i = 0; i < mArmJoints.size(); i++){
            mArmJoints[i] = _q[i];
        }
        mKinematicState->setJointGroupPositions(mJointsGroup, mArmJoints);
        if(mBackend != nullptr){
            mBackend->joints(std::vector<float>(mArmJoints.begin(), mArmJoints.end()));
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<double> Arm4DoF::joints() {
        mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
        return mArmJoints;
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
        Eigen::Affine3f affine3f(_position);
        bool foundIk = mKinematicState->setFromIK(mJointsGroup, affine3f.cast<double>() , 10, 0.1);
        if(foundIk){
            mKinematicState->copyJointGroupPositions(mJointsGroup, mArmJoints);
            _angles = mArmJoints;
            return true;
        }else{
            return false;
        }

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