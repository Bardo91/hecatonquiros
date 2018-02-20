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

#include <arm_controller/backends/Backend.h>

#include <string>
#include <Eigen/Eigen>
#include <math.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <thread>
#include <chrono>

namespace hecatonquiros{
    class Arm4DoF {
    public:
        /// Constructor 
        Arm4DoF(const Backend::Config &_config);
		// {
			// std::string mArmPrefix = "";
			// ros::NodeHandle n;
			// ros::Publisher mJointPublisher = n.advertise<sensor_msgs::JointState>(mArmPrefix+"/joint_states", 1000);

			// robot_model_loader::RobotModelLoader robot_model_loader("/robot_description");
			// robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
			// ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

			// // Using the :moveit_core:`RobotModel`, we can construct a
			// // :moveit_core:`RobotState` that maintains the configuration
			// // of the robot. We will set all joints in the state to their
			// // default values. We can then get a
			// // :moveit_core:`JointModelGroup`, which represents the robot
			// // model for a particular group, e.g. the "right_arm" of the PR2
			// // robot.
			// robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
			// kinematic_state->setToDefaultValues();
			// const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm4dof");

			// const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

			// while(ros::ok()){
			// 	// Get Joint Values
			// 	// ^^^^^^^^^^^^^^^^
			// 	// We can retreive the current set of joint values stored in the state for the right arm.
			// 	std::vector<double> joint_values;
			// 	// Joint Limits
			// 	// ^^^^^^^^^^^^
			// 	// setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
			// 	/* Set one joint in the right arm outside its joint limit */
			// 	kinematic_state->setToRandomPositions(joint_model_group);
			// 	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			// 	for (std::size_t i = 0; i < joint_names.size(); ++i)
			// 	{
			// 		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			// 	}
			// 	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

			// 	/* Enforce the joint limits for this state and check again*/
			// 	kinematic_state->enforceBounds();
			// 	ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

			// 	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			// 	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("arm_2");

			// 	/* Print end-effector pose. Remember that this is in the model frame */
			// 	ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
			// 	ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

			// 	sensor_msgs::JointState j_msg;

			// 	int counter = 0;
			// 	while(counter < 100){
			// 		j_msg.header.stamp = ros::Time::now();		
			// 		j_msg.header.seq=0;
			// 		j_msg.header.frame_id=mArmPrefix;
			// 		j_msg.position.push_back (joint_values[0]);
			// 		j_msg.position.push_back (joint_values[1]);
			// 		j_msg.position.push_back (joint_values[2]);   
			// 		j_msg.name.push_back (mArmPrefix+"arm_0_bottom_joint");
			// 		j_msg.name.push_back (mArmPrefix+"arm_1_joint");
			// 		j_msg.name.push_back (mArmPrefix+"arm_2_joint");
					
			// 		mJointPublisher.publish(j_msg);   
			// 		std::this_thread::sleep_for(std::chrono::milliseconds(30));
			// 		counter++;
			// 	}
				
			// 	// Forward Kinematics
			// 	// ^^^^^^^^^^^^^^^^^^
			// 	// Now, we can compute forward kinematics for a set of random joint
			// 	// values. Note that we would like to find the pose of the
			// 	// "r_wrist_roll_link" which is the most distal link in the
				
			// 	// Inverse Kinematics
			// 	// ^^^^^^^^^^^^^^^^^^
			// 	// We can now solve inverse kinematics (IK) for the right arm of the
			// 	// PR2 robot. To solve IK, we will need the following:
			// 	// * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain):
			// 	// end_effector_state that we computed in the step above.
			// 	// * The number of attempts to be made at solving IK: 5
			// 	// * The timeout for each attempt: 0.1 s
			// 	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
			// 	// Now, we can print out the IK solution (if found):
			// 	if (found_ik)
			// 	{
			// 		std::cout << "Inverse kinematic" << std::endl;
			// 	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			// 	for (std::size_t i = 0; i < joint_names.size(); ++i)
			// 	{
			// 		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			// 	}
			// 	}
			// 	else
			// 	{
			// 	ROS_INFO("Did not find IK solution");
			// 	}
			// 		std::cout << "-------------------------------------" << std::endl;
			// }
		//}
        /// Send robot to home.
        void home();

        /// Joints in radians
        void joints(std::vector<double>);
        std::vector<double> joints();

        /// Position in cartesian coordinates (meters)
        void position(Eigen::Matrix4f _position, float _wirst = 0);
        Eigen::Matrix4f position();

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Matrix4f _position);

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Matrix4f _position, std::vector<Eigen::Matrix4f> &_transformations);

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Matrix4f _position, std::vector<double> &_angles);

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Matrix4f _position, std::vector<double> &_angles, std::vector<Eigen::Matrix4f> &_transformations);

        /// Compute DK and check if valid
        bool directKinematic(const std::vector<double> &_angles, std::vector<Eigen::Matrix4f> &_transformations);

        //void lastTransformations(Eigen::Matrix4f &_t01, Eigen::Matrix4f &_t12, Eigen::Matrix4f &_t23, Eigen::Matrix4f &_t34);
        void lastTransformations(Eigen::Matrix4f &_t0, Eigen::Matrix4f &_t1, Eigen::Matrix4f &_t2);

        void closeClaw();
        void openClaw();
        void stopClaw();

        bool isInit() const;
    private:
        Backend *mBackend;

        std::vector<double> mArmJoints;

        robot_model_loader::RobotModelLoader *mRobotModelLoader;
        robot_model::RobotModelPtr mKinematicModel;
        robot_state::RobotStatePtr mKinematicState;
        robot_state::JointModelGroup *mJointsGroup;
        std::vector<std::string> mJointNames;
        float   mHome1 = 0*M_PI/180.0,
                mHome2 = 0*M_PI/180.0,
                mHome3 = 90*M_PI/180.0,
                mHome4 = 90*M_PI/180.0,
                mHome5 = 90*M_PI/180.0;

        int mArmId = 1;
    };
}