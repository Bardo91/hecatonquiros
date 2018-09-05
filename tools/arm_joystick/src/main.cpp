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
//#include <Joystick.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <serial/serial.h> 

//std::mutex armLock;
bool usingBothArms = false;
bool usingRight = true;

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "arm_joystick");

	hecatonquiros::Backend::Config backendConfig1;
	hecatonquiros::Backend::Config backendConfig2;
		
	int mode = 0;
	std::cout << "Press 1 for gazebo backend, 2 for arduino backend and 3 for feetech backend and 4 for no backend: " <<std::endl;
	std::cin >> mode;

	if(mode == 1){	// GAZEBO
		backendConfig1.type = hecatonquiros::Backend::Config::eType::Gazebo; 
		backendConfig1.topic = "aeroarms/left"; 
		backendConfig1.armId =1;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Gazebo; 
		backendConfig2.topic = "aeroarms/right"; 
		backendConfig2.armId =2;
	}else if(mode == 2){	// ARDUINO
		std::cout << "Write serial port (default /dev/ttyACM0), type 1 for default: ";
		std::string serialPort;
		std::cin >> serialPort;
		if(serialPort == "1"){
			serialPort = "/dev/ttyACM0";
		}
		serial::Serial* arduinoCom = new serial::Serial(serialPort, 115200, serial::Timeout::simpleTimeout(1000));
		if (!arduinoCom->isOpen()) {
			std::cout << "Could not open serial port, exiting" << std::endl;
			return -1;
		}

		backendConfig1.type = hecatonquiros::Backend::Config::eType::Arduino; backendConfig1.sharedSerialPort = arduinoCom; backendConfig1.armId =1;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Arduino; backendConfig2.sharedSerialPort = arduinoCom; backendConfig2.armId =2;
	}else if(mode == 3){	// feetech
		std::cout << "Write serial port (default/dev/ttyUSB0), type 1 for default: ";
		std::string serialPort;
		std::cin >> serialPort;
		if(serialPort == "1"){
			serialPort = "/dev/ttyUSB0";
		}

		backendConfig1.configXML = "src/hecatonquiros/arm_controller/config/config_arm1.xml";
		backendConfig2.configXML = "src/hecatonquiros/arm_controller/config/config_arm2.xml";

		backendConfig1.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig1.port = serialPort; backendConfig1.armId =1;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Feetech; backendConfig2.port = serialPort; backendConfig2.armId =2;
		
	}else if(mode == 4){	// dummy
		backendConfig1.type = hecatonquiros::Backend::Config::eType::Dummy;
		backendConfig2.type = hecatonquiros::Backend::Config::eType::Dummy;
	}else{
		std::cout << "unrecognized mode, exiting" << std::endl;
        return -1;
	}
	
    hecatonquiros::ModelSolver::Config modelSolverConfig1;
    modelSolverConfig1.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    modelSolverConfig1.robotName = "left_arm";
    modelSolverConfig1.manipulatorName = "manipulator";
    modelSolverConfig1.robotFile = _argv[1];
    //modelSolverConfig1.environment = _argv[1];
    modelSolverConfig1.offset = {0.20,0.14,-0.04};
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(1*M_PI,  Eigen::Vector3f::UnitY());
	Eigen::Quaternionf q(m);
    modelSolverConfig1.rotation = {q.w(), q.x(), q.y(), q.z()};
    modelSolverConfig1.visualizer = true;

    hecatonquiros::ModelSolver::Config modelSolverConfig2;
    modelSolverConfig2.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    modelSolverConfig2.robotName = "right_arm";
    modelSolverConfig2.manipulatorName = "manipulator";
    modelSolverConfig2.robotFile = _argv[1];
    modelSolverConfig2.offset = {0.2,-0.2,-0.04};
    modelSolverConfig2.rotation = {q.w(), q.x(), q.y(), q.z()};
    modelSolverConfig2.visualizer = true;

	hecatonquiros::Arm4DoF leftArm(modelSolverConfig1, backendConfig1);
	hecatonquiros::Arm4DoF rightArm(modelSolverConfig2, backendConfig2);

	leftArm.home();
	rightArm.home();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	
	hecatonquiros::Arm4DoF *armInUse = &rightArm;
	std::cout << "USING RIGHT ARM" << &armInUse<<std::endl;



  	//Joystick joystick("/dev/input/js1");
	//std::thread joystickThread;
	//JoystickEvent joystickEvent;
	//if(joystick.isFound()){
    //        joystickThread = std::thread([&](){
    //				if (joystick.sample(&joystickEvent))
    //				{
    					//if (joystickEvent.isButton()){
						//	if(joystickEvent.number == ){
						//		usingBothArms = true;
						//	}else if(joystickEvent.number == ){
						//		usingBothArms = false;
						//		if(usingRight){
						//			usingRight = false;
						//			armInUse = &leftArm;
						//			std::cout << "USING LEFT ARM" << std::endl;
						//		}else{
						//			usingRight = true;
						//			armInUse = &rightArm;
						//			std::cout << "USING RIGHT ARM" << std::endl;
						//		}
						//	}else if(joystickEvent.number == ){
						//		if(usingBothArms){
						//			leftArm.openClaw();
						//			rightArm.openClaw();
						//		}else{
						//			armInUse->openClaw();
						//		}
						//	}else if(joystickEvent.number == ){
						//		if(usingBothArms){
						//			leftArm.closeClaw();
						//			rightArm.closeClaw();
						//		}else{
						//			armInUse->closeClaw();
						//		}
						//	}else if(joystickEvent.number == ){
						//		if(usingBothArms){
						//			leftArm.home();
						//			rightArm.home();
						//		}else{
						//			armInUse->home();
						//		}
						//	}else if(joystickEvent.number == ){
						//		if(usingBothArms){
						//			std::cout << "Pose Left" << std::endl;
						//			auto poseL = leftArm.pose();
						//			std::cout << poseL << std::endl;
						//			std::cout << "Pose Right" << std::endl;
						//			auto poseR = rightArm.pose();
						//			std::cout << poseR << std::endl;
						//		}else{
						//			std::cout << "Pose only arm in use" << std::endl;
						//			auto pose = armInUse->pose();
						//			std::cout << pose << std::endl;
						//		}
						//	}
    				//
    				  	//}else if (joystickEvent.isAxis()){
						//	if(joystickEvent.number == ){
						//		
						//	}else if(joystickEvent.number == ){
						//		
						//	}else if(joystickEvent.number == ){
						//		
						//	}
    //				  	}
    //				}
    //                std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //            }
    //        });
    //}else{
    //    std::cout << "Joystick not connected!" << std::endl;
    //}

	while(true){

		if(usingBothArms){




		}else{

			
			//switch(c){
				//case 'p':
				//{
				//	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
				//	pose(0,3) = ;
				//	pose(1,3) = ;
				//	pose(2,3) = ;
				//	
				//	std::vector<float> joints;
				//	auto start = std::chrono::high_resolution_clock::now();
				//	if(armInUse->checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
				//		armInUse->joints(joints, true);
				//	}else{
				//		std::cout << "Not found IK" << std::endl;
				//	}
				//	auto end = std::chrono::high_resolution_clock::now();
				//	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
				//	std::cout << duration.count() << std::endl;
				//	break;
			//	}


		}
	}
}


