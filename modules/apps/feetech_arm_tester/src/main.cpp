//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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


#include <hecatonquiros/Arm4DoF.h>
#define HAS_OPENRAVE
#include <hecatonquiros/model_solvers/ModelSolverOpenRave.h>
#include <Eigen/QR>
#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <serial/serial.h> 

#include <chrono>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "arm_tester");

	ros::AsyncSpinner spinner(4);
	spinner.start();

	hecatonquiros::Backend::Config backendConfig;
	backendConfig.configXML = _argv[1];
	backendConfig.type = hecatonquiros::Backend::Config::eType::Feetech; 
	backendConfig.port = "/dev/ttyACM0"; 
	backendConfig.armId =2;

	srand(time(NULL));

    hecatonquiros::ModelSolver::Config modelSolverConfig;
    modelSolverConfig.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
	modelSolverConfig.robotName = "arm";
    modelSolverConfig.manipulatorName = "manipulator";
    modelSolverConfig.robotFile = _argv[2];
    modelSolverConfig.offset = {0.0,0.0,0.0};
	
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(1*M_PI,  Eigen::Vector3f::UnitY());
	Eigen::Quaternionf q(m);
    
	modelSolverConfig.rotation = {q.w(), q.x(), q.y(), q.z()};
    modelSolverConfig.visualizer = false;

	hecatonquiros::Arm4DoF arm(modelSolverConfig, backendConfig);

	arm.home();
	
	auto getTimeNow = [](){return std::chrono::high_resolution_clock::now();};
	auto sleepMillis = [](int _millis){std::this_thread::sleep_for(std::chrono::milliseconds(_millis));};

	auto tSafe = getTimeNow();
	auto tJoy = getTimeNow();
	bool goSafe = true;
	sensor_msgs::Joy lastJoy;
	ros::NodeHandle nh; 
	ros::Subscriber joySubscriber = nh.subscribe<sensor_msgs::Joy>("/flames/joy", 1, [&](const sensor_msgs::Joy::ConstPtr &_msg){
		lastJoy = *_msg;
		tJoy = getTimeNow();
	});

	ros::Subscriber safeKeeper = nh.subscribe<std_msgs::Bool>("/flames/keep_alive", 1, [&](const std_msgs::Bool::ConstPtr &_msg){
		tSafe = getTimeNow();
	});

	sleepMillis(500);
	tSafe = getTimeNow();


	while(ros::ok()){
		if(goSafe){
			arm.home();
			sleepMillis(100);
		}else{
			if(std::chrono::duration_cast<std::chrono::milliseconds>(getTimeNow() - tJoy).count() < 200){
				if(lastJoy.buttons[3] == 1){
					arm.home();
					sleepMillis(500);
				}else if(lastJoy.axes.size() > 0){
					auto position = arm.position();
					
					position[0] += lastJoy.axes[1]*0.005;
					position[1] += lastJoy.axes[3]*0.005;
					position[2] += lastJoy.axes[4]*0.005;

					arm.position(position, -40, true);
					// std::vector<float> joints;
					// arm.jacobianStep(position, joints, 0.1);
					// arm.joints(joints, true);
					// std::cout << "moving: " << position << std::endl;
				}
			}
		}

		auto timeCheck = std::chrono::duration_cast<std::chrono::milliseconds>(getTimeNow() - tSafe).count();
		if(timeCheck > 500){
			goSafe = true;
		}
		else{
			goSafe = false;
		}
		sleepMillis(30);
	}
}