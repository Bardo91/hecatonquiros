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


#include <arm_controller/Positioner.h>
#include <iostream>

//---------------------------------------------------------------------------------------------------------------------
Positioner::Positioner(std::string _port, int _baudrate) {
    mArduinoCom = new serial::Serial(_port, _baudrate, serial::Timeout::simpleTimeout(1000));
    init();
}

//---------------------------------------------------------------------------------------------------------------------
Positioner::Positioner(serial::Serial *_serialPort) {
    mArduinoCom = _serialPort;
    init();
}

//---------------------------------------------------------------------------------------------------------------------
void Positioner::init() {
    mP0 = {-230.0f/2,   230.0f/2}; //{-230.0f/2,   230.0f/2};
    mP1 = {-130.0f,     125.0f }; //{-120.0f, 140.0f };
    mP2 = {-110.0f,     115.0f }; //{-130.0f, 115.0f };
    mP3 = { -115.0f,    115.0f }; //{ -130.0f, 135.0f };
    mP4 = { -130.0f,    135.0f }; //{ -130.0f, 135.0f };

    mSerialThread = std::thread([&]() {
        std::string data = "";
        while (mRun) {
            mSecureRead.lock();
            mArduinoCom->write("p\r\n");
            std::string data = mArduinoCom->readline();
            try {
                if (data.find_first_of(",") != data.npos) {
                    mJ4 = atof(data.substr(0, data.find_first_of(",")).c_str());
                    data = data.substr(data.find_first_of(",") + 1, data.size());
                    mJ3 = atof(data.substr(0, data.find_first_of(",")).c_str());
                    data = data.substr(data.find_first_of(",") + 1, data.size());
                    mJ2 = atof(data.substr(0, data.find_first_of(",")).c_str());
                    data = data.substr(data.find_first_of(",") + 1, data.size());
                    mJ1 = atof(data.substr(0, data.find_first_of(",")).c_str());
                    data = data.substr(data.find_first_of(",") + 1, data.size());
                    mJ0 = atof(data.c_str());
                }
            }
            catch (int _e){
                std::cout << "Read exception" << std::endl;
            }
            mSecureRead.unlock();
            //std::cout << mJ0 << ", " << mJ1 << ", " << mJ2 << ", " << mJ3 << ", " << mJ4 << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::cout << "Initialized Positioner" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
bool Positioner::close() {
	mRun = false;
	while (!mSerialThread.joinable()) {}
	mSerialThread.join();

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
void Positioner::baseToHand(float &_x, float &_y, float &_z){
    std::vector<float> vals = angles();
 

    mT01 <<	cos(vals[0]), 0, sin(vals[0]), 0,
            sin(vals[0]), 0, -cos(vals[0]), 0,
			0, 1, 0, cL01,
			0, 0, 0, 1;
   
    mT12 <<	cos(M_PI / 2 + vals[1]), -sin(M_PI / 2 + vals[1]), 0, (cL12 * cos(M_PI / 2 + vals[1])),
            sin(M_PI / 2 + vals[1]), cos(M_PI / 2 + vals[1]), 0, (cL12 * sin(M_PI / 2 + vals[1])),
			0, 0, 1, 0,
			0, 0, 0, 1;

    mT23<<	cos(vals[2]), -sin(vals[2]), 0, (cL23 * cos(vals[2])),
            sin(vals[2]), cos(vals[2]), 0, (cL23 * sin(vals[2])),
			0, 0, 1, 0,
			0, 0, 0, 1;

  
    mT34 <<	cos(vals[3]), 0, sin(vals[3]), (cL34 * cos(vals[3])),
            sin(vals[3]), 0, -cos(vals[3]), (cL34 * sin(vals[3])),
			0, 1, 0, 0,
			0, 0, 0, 1;

    mT4f << cos(vals[4]), -sin(vals[4]), 0, (cL4f * cos(vals[4])),
            sin(vals[4]), cos(vals[4]), 0, (cL4f * sin(vals[4])),
			0, 0, 1, 0,
			0, 0, 0, 1;

	Eigen::Matrix4f finalT = mT01*mT12*mT23*mT34*mT4f;

	_x = finalT(0,3);
	_y = finalT(1,3);
	_z = finalT(2,3);
  
}

//---------------------------------------------------------------------------------------------------------------------
void Positioner::handToBase(float &_x, float &_y, float &_z){
  
}

//---------------------------------------------------------------------------------------------------------------------
void Positioner::rawJoints(float &_j0, float &_j1, float &_j2, float &_j3, float &_j4){
	mSecureRead.lock();
	_j0 = mJ0;
	_j1 = mJ1;
	_j2 = mJ2;
	_j3 = mJ3;
	_j4 = mJ4;
	mSecureRead.unlock();
}

//---------------------------------------------------------------------------------------------------------------------
std::vector<float> Positioner::angles(){
    std::vector<float> vals(5);
    mSecureRead.lock();
    vals[0] = mP0.valToAngle(mJ0) / 180.0f*M_PI;//((mJ0 - 512.0f) / 512.0f) * (135.0f / 180.0f * M_PI);
    vals[1] = mP1.valToAngle(mJ1) / 180.0f*M_PI;//((mJ1 - 512.0f) / 512.0f) * (135.0f / 180.0f * M_PI);
    vals[2] = mP2.valToAngle(mJ2) / 180.0f*M_PI;//((mJ2 - 512.0f) / 512.0f) * (135.0f / 180.0f * M_PI);
    vals[3] = mP3.valToAngle(mJ3) / 180.0f*M_PI;//((mJ3 - 512.0f) / 512.0f) * (135.0f / 180.0f * M_PI);
    vals[4] = -mP4.valToAngle(mJ4) / 180.0f*M_PI;//((mJ3 - 512.0f) / 512.0f) * (135.0f / 180.0f * M_PI);
	mSecureRead.unlock();
    return vals;
}

//---------------------------------------------------------------------------------------------------------------------
void Positioner::lastTransforms(Eigen::Matrix4f & _T01, Eigen::Matrix4f & _T12, Eigen::Matrix4f & _T23, Eigen::Matrix4f & _T34, Eigen::Matrix4f & _T4f) {
	_T01 = mT01;
	_T12 = mT12;
	_T23 = mT23;
	_T34 = mT34;
	_T4f = mT4f;
}
