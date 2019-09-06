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


#include <hecatonquiros/Positioner.h>
#include <iostream>
#include <chrono>

int calibrateJoint(hecatonquiros::Positioner &_dockingTool);
int readAll(hecatonquiros::Positioner &_dockingTool);

int main(int _argc, char **_argv) {
	hecatonquiros::Positioner dockingTool("/dev/ttyACM0", 115200);
	std::this_thread::sleep_for(std::chrono::seconds(2));


	dockingTool.toggleLock();

	auto menuLambda = [&](){
		std::cout << "Choose an action:" << std::endl;
		std::cout << "\t 0) exit" << std::endl;
		std::cout << "\t 1) read all" << std::endl;
		std::cout << "\t 2) calibrate joint" << std::endl;
		int cmd;
		std::cin >> cmd;
		return cmd;
	};

	bool run = true;
	while(run){
		int cmd = menuLambda();
		switch(cmd){
			case 0:
				run = false;
				break;
			case 1:
				readAll(dockingTool);
				break;
			case 2:
				calibrateJoint(dockingTool);
				break;
			default:
				run = false;
		}
	}
	
}


int calibrateJoint(hecatonquiros::Positioner &_dockingTool){
	std::cout << "Which joint do you want to calibrate? (0-4)";
	int joint;
	std::cin >> joint;
	getchar();
	auto chrono_now = [&](){return std::chrono::high_resolution_clock::now();};

	auto readPosition = [&](float _angle){
		std::cout << "Please hold the joint at " << _angle*180.0/M_PI << " degrees and press enter, then hold for 2 seconds";
		getchar();
		std::vector<std::pair<float,float>> y;
		
		std::vector<float> angles(5);
		auto t0 = chrono_now();
		std::string str = "";
		while(std::chrono::duration_cast<std::chrono::seconds>(chrono_now()- t0).count() < 2){	
			angles = _dockingTool.rawJoints();
			y.push_back(std::pair<float,float>(_angle,angles[joint]));
			for(int i = 0; i < str.size(); i++) std::cout << "\b";
			str = std::to_string(angles[joint]);
			std::cout  << str;
		}
		return y;
	};

	auto datam90 = readPosition(-90.0/180.0*M_PI);
	auto data0 = readPosition(0);
	auto data90 = readPosition(90.0/180.0*M_PI);


	std::cout << "Done reading, start computing"<< std::endl;
	
	std::vector<std::pair<float,float>> allData;
	allData.insert(allData.end(), datam90.begin(), datam90.end());
	allData.insert(allData.end(), data0.begin(), data0.end());
	allData.insert(allData.end(), data90.begin(), data90.end());

	auto sumX = [&](std::vector<std::pair<float,float>> &_data){
		float res = 0; 
		for(auto &pair:_data) res += pair.first;
		return res;
	};

	auto sumY = [&](std::vector<std::pair<float,float>> &_data){
		float res = 0; 
		for(auto &pair:_data) res += pair.second;
		return res;
	};

	auto sumX2 = [&](std::vector<std::pair<float,float>> &_data){
		float res = 0; 
		for(auto &pair:_data) res += pair.first*pair.first;
		return res;
	};

	auto sumXY = [&](std::vector<std::pair<float,float>> &_data){
		float res = 0; 
		for(auto &pair:_data) res += pair.first*pair.second;
		return res;
	};

	auto calcSlope = [&](std::vector<std::pair<float,float>> &_data){
		int n = _data.size();
		float s = (n*sumXY(_data) - sumX(_data)*sumY(_data))/(n*sumX2(_data) - sumX(_data)*sumX(_data));
		return s;
	};

	auto calcIntercept = [&](std::vector<std::pair<float,float>> &_data, float _slope){
		int n = _data.size();
		return (sumY(_data) - _slope*sumX(_data))/n;
	};


	float slope = calcSlope(allData);
	float intercept = calcIntercept(allData, slope);

	std::cout << "Equation of joint " << joint << "is y="<<slope<<"x+"<<intercept <<std::endl;

}

int readAll(hecatonquiros::Positioner &_dockingTool){
	std::cout << "Introduced seconds to read: ";
	int seconds;
	std::cin >> seconds;
	
	std::vector<float> angles(5);
	auto chrono_now = [&](){return std::chrono::high_resolution_clock::now();};
	auto t0 = chrono_now();
	while(std::chrono::duration_cast<std::chrono::seconds>(chrono_now()- t0).count() < seconds){
		angles = _dockingTool.angles();

		std::cout << "Angles: ";
		std::cout << 	angles[0]/M_PI*180.0 << ", " << 
						angles[1]/M_PI*180.0 << ", " << 
						angles[2]/M_PI*180.0 << ", " << 
						angles[3]/M_PI*180.0 << ", " << 
						angles[4]/M_PI*180.0 << std::endl;

		std::cout << "Accel: ";
		std::cout << _dockingTool.accelerationVector().transpose() << std::endl;
		std::cout << "---------------------------" << std::endl;
	}
}