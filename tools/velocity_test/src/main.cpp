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

#include <string>
#include <thread>
#include <fstream>
#include <chrono>
#include <mutex>

#include <math.h>
#include <cmath>

#include <hecatonquiros/backends/dep/SCServo.h>

//-----------------------------------------------------------------------------------------------------------------
int mapAngleToVal(float _minAngle, float _maxAngle, float _angle){
    return (_angle-_minAngle)/(_maxAngle-_minAngle)*(1023-0);
}

//-----------------------------------------------------------------------------------------------------------------
float mapValToAngle(float _minAngle, float _maxAngle, int _val){
    return (_val-0.0)/(1023.0-0.0)*(_maxAngle-_minAngle) + _minAngle;
}

//-----------------------------------------------------------------------------------------------------------------
int main(int _argc, char** _argv){

    std::mutex guard;
    SCServo *servoDriver;

    std::string serialPort = "/dev/ttyUSB0";

    std::vector<std::pair<float, float> > minMaxValues1 = { {-109.376/180.0*M_PI, 106.492/180.0*M_PI},   
                                                            {-111.490/180.0*M_PI, 107.986/180.0*M_PI},
                                                            {-104.869/180.0*M_PI, 113.775/180.0*M_PI},
                                                            {-117.883/180.0*M_PI, 98.495/180.0*M_PI},
                                                            {-135.267/180.0*M_PI, 142.044/180.0*M_PI},
                                                            {-156.874/180.0*M_PI, 139.401/180.0*M_PI}};

    std::vector<float> offsetJoints1 = {-0.05, 0.087266463, 0.087266463, 0.0, 0.08726646259, 0.0};


    std::vector<std::pair<float, float> > minMaxValues2 = { {-112.737/180.0*M_PI, 100.442/180.0*M_PI},   
                                                            {-106.202/180.0*M_PI, 110.798/180.0*M_PI},
                                                            {-103.401/180.0*M_PI, 116.820/180.0*M_PI},
                                                            {-105.499/180.0*M_PI, 110.352/180.0*M_PI},
                                                            {-130.213/180.0*M_PI, 142.940/180.0*M_PI},
                                                            {-153.010/180.0*M_PI, 146.854/180.0*M_PI}};

    std::vector<float> offsetJoints2 = {-0.05, 0.13089969389, 0.08726646259, -0.08726646259, 0.0, 0.0};

    std::vector<std::pair<float, float> > minMaxValues3 = { {-109.376/180.0*M_PI, 106.492/180.0*M_PI},   
                                                            {-111.490/180.0*M_PI, 107.986/180.0*M_PI},
                                                            {-104.869/180.0*M_PI, 113.775/180.0*M_PI},
                                                            {-117.883/180.0*M_PI, 98.495/180.0*M_PI},
                                                            {-135.267/180.0*M_PI, 142.044/180.0*M_PI},
                                                            {-156.874/180.0*M_PI, 139.401/180.0*M_PI},
                                                            {-112.737/180.0*M_PI, 100.442/180.0*M_PI},   
                                                            {-106.202/180.0*M_PI, 110.798/180.0*M_PI},
                                                            {-103.401/180.0*M_PI, 116.820/180.0*M_PI},
                                                            {-105.499/180.0*M_PI, 110.352/180.0*M_PI},
                                                            {-130.213/180.0*M_PI, 142.940/180.0*M_PI},
                                                            {-153.010/180.0*M_PI, 146.854/180.0*M_PI}};

    std::vector<float> offsetJoints3 = {-0.05, 0.087266463, 0.087266463, 0.0, 0.08726646259, 0.0, -0.05, 0.13089969389, 0.08726646259, -0.08726646259, 0.0, 0.0};

    servoDriver =  new SCServo(serialPort);
    if(servoDriver->isConnected()){

        // ARM 1
        auto t0 = std::chrono::high_resolution_clock::now();
        std::vector<float> joints1 = {0.0, 0.0, 0.0, 0.0};
        unsigned char idn1 = joints1.size();
        unsigned char id1[idn1];
        unsigned short pos1[idn1], tim1[idn1], spee1[idn1];
        for(unsigned i = 0; i < joints1.size(); i++){
            id1[i] = 10 + i + 1;
            pos1[i] = mapAngleToVal(minMaxValues1[i].first, minMaxValues1[i].second, joints1[i] + offsetJoints1[i]);
            tim1[i] = 500;
            spee1[i] = 0;
        }
        guard.lock(); 
        servoDriver->SyncWritePos(id1, idn1, pos1, tim1, spee1);
        guard.unlock();
        auto t1 = std::chrono::high_resolution_clock::now();
		float duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
        std::cout << "TIME SEND JOINTS ARM1: " << duration1 << std::endl;
 
        // ARM 2
        auto t2 = std::chrono::high_resolution_clock::now();
        std::vector<float> joints2 = {0.0, 0.0, 0.0, 0.0};
        unsigned char idn2 = joints2.size();
        unsigned char id2[idn2];
        unsigned short pos2[idn2], tim2[idn2], spee2[idn2];
        for(unsigned j = 0; j < joints1.size(); j++){
            id2[j] = 20 + j + 1;
            pos2[j] = mapAngleToVal(minMaxValues2[j].first, minMaxValues2[j].second, joints2[j] + offsetJoints2[j]);
            tim2[j] = 500;
            spee2[j] = 0;
        }
        guard.lock(); 
        servoDriver->SyncWritePos(id2, idn2, pos2, tim2, spee2);
        guard.unlock();
        auto t3 = std::chrono::high_resolution_clock::now();
		float duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count();
        std::cout << "TIME SEND JOINTS ARM2: " << duration2 << std::endl;
        
        // BOTH ARMS
        auto t4 = std::chrono::high_resolution_clock::now();
        std::vector<float> joints3 = {0.0, 0.0, 0.0, 0.0};
        std::vector<float> joints4 = {0.0, 0.0, 0.0, 0.0};
        unsigned char idn3 = joints3.size() + joints4.size();
        unsigned char id3[idn3];
        unsigned short pos3[idn3], tim3[idn3], spee3[idn3];
        for(unsigned k = 0; k < idn3; k++){
            if(k < joints3.size()){
                id3[k] = 10 + k + 1;
                pos3[k] = mapAngleToVal(minMaxValues3[k].first, minMaxValues3[k].second, joints3[k] + offsetJoints3[k]);
                tim3[k] = 500;
                spee3[k] = 0;
            }else{
                id3[k] = 20 + k + 1;
                pos3[k] = mapAngleToVal(minMaxValues3[k].first, minMaxValues3[k].second, joints4[k] + offsetJoints3[k]);
                tim3[k] = 500;
                spee3[k] = 0;
            }
            
        }
        guard.lock(); 
        servoDriver->SyncWritePos(id3, idn3, pos3, tim3, spee3);
        guard.unlock();
        auto t5 = std::chrono::high_resolution_clock::now();
		float duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(t5-t4).count();
        std::cout << "TIME SEND JOINTS ARM1 & ARM2: " << duration3 << std::endl;


        //////////////////////////////////////////////////
        // READ ARM1
        auto t6 = std::chrono::high_resolution_clock::now();
        std::vector<float> joints_read1;
        for(unsigned l = 0; l < 4; l++){
            guard.lock();
            int val = servoDriver->ReadPos(10 + l + 1);
            guard.unlock();
            if(val < 0 || val > 1024){
                joints_read1.push_back(-1);
            }else{
                joints_read1.push_back( mapValToAngle( minMaxValues1[l].first, 
                                                minMaxValues1[l].second,
                                                val) - offsetJoints1[l]);
            }
        }
        auto t7 = std::chrono::high_resolution_clock::now();
		float duration4 = std::chrono::duration_cast<std::chrono::milliseconds>(t7-t6).count();
        std::cout << "TIME READ JOINTS ARM1: " << duration4 << std::endl;

        // READ ARM2
        auto t8 = std::chrono::high_resolution_clock::now();
        std::vector<float> joints_read2;
        for(unsigned m = 0; m < 4; m++){
            guard.lock();
            int val = servoDriver->ReadPos(10 + m + 1);
            guard.unlock();
            if(val < 0 || val > 1024){
                joints_read2.push_back(-1);
            }else{
                joints_read2.push_back( mapValToAngle( minMaxValues2[m].first, 
                                                minMaxValues2[m].second,
                                                val) - offsetJoints2[m]);
            }
        }
        auto t9 = std::chrono::high_resolution_clock::now();
		float duration5 = std::chrono::duration_cast<std::chrono::milliseconds>(t9-t8).count();
        std::cout << "TIME READ JOINTS ARM2: " << duration5 << std::endl;



    }

    return 0;
}
