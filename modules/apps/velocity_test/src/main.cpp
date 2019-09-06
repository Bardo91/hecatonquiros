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

#include <hecatonquiros/backends/Backend.h> 

#include "Graph2d.h"

std::vector<std::vector<double>> targetJoints(4);
std::vector<std::vector<double>> currentJoints(4);

std::chrono::high_resolution_clock::time_point veryT0;
std::chrono::high_resolution_clock::time_point veryT1;

float durWrite1, durWrite2 = 0;
float durRead1, durRead2 = 0;

float counter = 0;
float counter2 = 0;
float speedCounter = 1;
float speedCounter2 = 1;

//-----------------------------------------------------------------------------------------------------------------
int mapAngleToVal(float _minAngle, float _maxAngle, float _angle){
    return (_angle-_minAngle)/(_maxAngle-_minAngle)*(1023-0);
}

//-----------------------------------------------------------------------------------------------------------------
float mapValToAngle(float _minAngle, float _maxAngle, int _val){
    return (_val-0.0)/(1023.0-0.0)*(_maxAngle-_minAngle) + _minAngle;
}

//-----------------------------------------------------------------------------------------------------------------
void threadArm1(){
    
    hecatonquiros::Backend::Config bc1; 
    bc1.configXML = "/home/belerofonte/programming/catkin_hecatonquiros/src/hecatonquiros/arm_controller/config/config_arm1.xml"; 
    bc1.type = hecatonquiros::Backend::Config::eType::FeetechQueueThread; 
    bc1.port = "/dev/ttyUSB0";
    bc1.armId = 1;

    hecatonquiros::Backend * backend1 = hecatonquiros::Backend::create(bc1);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    for(;;){

        veryT0 = std::chrono::high_resolution_clock::now();

        // WRITE ARMS
        auto t4 = std::chrono::high_resolution_clock::now();
        std::vector<float> joints1 = {0.0*M_PI/180.0, (counter/50)*45.0*M_PI/180.0, (counter2/50)*45.0*M_PI/180.0, 0.0*M_PI/180.0};
        
        backend1->joints(joints1, true);
        
        auto t5 = std::chrono::high_resolution_clock::now();
        float duration3 = std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count();
        //std::cout << "Time send joints ARM 1: " << std::to_string(duration3) << std::endl;
        durWrite1 = duration3;

        //////////////////////////////////////////////////
        // READ ARMS
        auto t10 = std::chrono::high_resolution_clock::now();

        std::vector<float> joints_read1 = backend1->joints(joints1.size(), true);

        targetJoints[0].push_back(0.0*M_PI/180.0);
        targetJoints[1].push_back((counter/50)*45.0*M_PI/180.0);
        targetJoints[2].push_back((counter2/50)*45.0*M_PI/180.0);
        targetJoints[3].push_back(0.0*M_PI/180.0);

        currentJoints[0].push_back(joints_read1[0]);
        currentJoints[1].push_back(joints_read1[1]);
        currentJoints[2].push_back(joints_read1[2]);
        currentJoints[3].push_back(joints_read1[3]);

        auto t11 = std::chrono::high_resolution_clock::now();
        float duration6 = std::chrono::duration_cast<std::chrono::microseconds>(t11-t10).count();
        //std::cout << "Time read joints ARM 1: " << std::to_string(duration6) << std::endl;
        durRead1 = duration6;

    }

}

//-----------------------------------------------------------------------------------------------------------------
void threadArm2(){
    
    hecatonquiros::Backend::Config bc2; 
    bc2.configXML = "/home/belerofonte/programming/catkin_hecatonquiros/src/hecatonquiros/arm_controller/config/config_arm2.xml"; 
    bc2.type = hecatonquiros::Backend::Config::eType::FeetechQueueThread; 
    bc2.port = "/dev/ttyUSB0";
    bc2.armId = 2;

    hecatonquiros::Backend * backend2 = hecatonquiros::Backend::create(bc2);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    for(;;){

        // WRITE ARMS
        auto t4 = std::chrono::high_resolution_clock::now();
        std::vector<float> joints2 = {0.0*M_PI/180.0, (counter/50)*45.0*M_PI/180.0, (counter2/50)*45.0*M_PI/180.0, 0.0*M_PI/180.0};
        
        backend2->joints(joints2, true);
        
        auto t5 = std::chrono::high_resolution_clock::now();
        float duration3 = std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count();
        //std::cout << "Time send joints ARM 2: " << std::to_string(duration3) << std::endl;
        durWrite2 = duration3;

        //////////////////////////////////////////////////
        // READ ARMS
        auto t10 = std::chrono::high_resolution_clock::now();

        std::vector<float> joints_read2 = backend2->joints(joints2.size(), true);

        auto t11 = std::chrono::high_resolution_clock::now();
        float duration6 = std::chrono::duration_cast<std::chrono::microseconds>(t11-t10).count();
        //std::cout << "Time read joints ARM 2: " << std::to_string(duration6) << std::endl;
        durRead2 = duration6;

        veryT1 = std::chrono::high_resolution_clock::now();

    }

}


//-----------------------------------------------------------------------------------------------------------------
int main(int _argc, char** _argv){

    std::vector<double> timeWrite;
    std::vector<double> timeRead;
    std::vector<double> timeAll;

    rgbd::Graph2d plotTimes("times");
    rgbd::Graph2d plotJoints("Joints");

    std::thread arm1(threadArm1);
    std::thread arm2(threadArm2);

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    for(;;){
        counter +=speedCounter;
        counter2 +=speedCounter2*2;
        if(counter > 49){
            speedCounter = -1;
        }if(counter < -49){
            speedCounter = 1;
        }
        if(counter2 > 49){
            speedCounter2 = -1;
        }if(counter2< -49){
            speedCounter2 = 1;
        }
        
        //////////////////////////////////////////////////

        float wholeDuration = std::chrono::duration_cast<std::chrono::microseconds>(veryT1-veryT0).count();
        timeAll.push_back(wholeDuration);

        float durTotWrite = durWrite1 + durWrite2;
        timeWrite.push_back(durTotWrite);
        std::cout << "Time send both ARMS: " << std::to_string(durTotWrite) << std::endl;

        float durTotRead = durRead1 + durRead2;
        timeRead.push_back(durTotRead);
        std::cout << "Time read both ARMS: " << std::to_string(durTotRead) << std::endl;

        int len = timeRead.size() < 100? timeRead.size():100;

        auto allSubset = std::vector<double>(timeAll.end()-len, timeAll.end());
        auto writeSubset = std::vector<double>(timeWrite.end()-len, timeWrite.end());
        auto readSubset = std::vector<double>(timeRead.end()-len, timeRead.end());

        plotTimes.clean();
        plotTimes.draw(allSubset,     255,100,100, rgbd::Graph2d::eDrawType::Lines);
        plotTimes.draw(writeSubset,   100,255,100, rgbd::Graph2d::eDrawType::Lines);
        plotTimes.draw(readSubset,    100,100,255, rgbd::Graph2d::eDrawType::Lines);
        plotTimes.show();
        
        plotJoints.clean();
        plotJoints.draw(std::vector<double>(targetJoints[0].end()-len, targetJoints[0].end()), 255,100,100, rgbd::Graph2d::eDrawType::Circles);
        plotJoints.draw(std::vector<double>(targetJoints[1].end()-len, targetJoints[1].end()), 255,255,100, rgbd::Graph2d::eDrawType::Circles);
        plotJoints.draw(std::vector<double>(targetJoints[2].end()-len, targetJoints[2].end()), 255,100,255, rgbd::Graph2d::eDrawType::Circles);
        plotJoints.draw(std::vector<double>(targetJoints[3].end()-len, targetJoints[3].end()), 100,100,255, rgbd::Graph2d::eDrawType::Circles);

        plotJoints.draw(std::vector<double>(currentJoints[0].end()-len, currentJoints[0].end()), 255,100,100, rgbd::Graph2d::eDrawType::Lines);
        plotJoints.draw(std::vector<double>(currentJoints[1].end()-len, currentJoints[1].end()), 255,255,100, rgbd::Graph2d::eDrawType::Lines);
        plotJoints.draw(std::vector<double>(currentJoints[2].end()-len, currentJoints[2].end()), 255,100,255, rgbd::Graph2d::eDrawType::Lines);
        plotJoints.draw(std::vector<double>(currentJoints[3].end()-len, currentJoints[3].end()), 100,100,255, rgbd::Graph2d::eDrawType::Lines);
        plotJoints.show();

        cv::waitKey(10);

    // std::mutex guard;
    // SCServo *servoDriver;

    // std::string serialPort = "/dev/ttyUSB0";

    // std::vector<std::pair<float, float> > minMaxValues1 = { {-109.376/180.0*M_PI, 106.492/180.0*M_PI},   
    //                                                         {-111.490/180.0*M_PI, 107.986/180.0*M_PI},
    //                                                         {-104.869/180.0*M_PI, 113.775/180.0*M_PI},
    //                                                         {-117.883/180.0*M_PI, 98.495/180.0*M_PI},
    //                                                         {-135.267/180.0*M_PI, 142.044/180.0*M_PI},
    //                                                         {-156.874/180.0*M_PI, 139.401/180.0*M_PI}};

    // std::vector<float> offsetJoints1 = {-0.05, 0.087266463, 0.087266463, 0.0, 0.08726646259, 0.0};


    // std::vector<std::pair<float, float> > minMaxValues2 = { {-112.737/180.0*M_PI, 100.442/180.0*M_PI},   
    //                                                         {-106.202/180.0*M_PI, 110.798/180.0*M_PI},
    //                                                         {-103.401/180.0*M_PI, 116.820/180.0*M_PI},
    //                                                         {-105.499/180.0*M_PI, 110.352/180.0*M_PI},
    //                                                         {-130.213/180.0*M_PI, 142.940/180.0*M_PI},
    //                                                         {-153.010/180.0*M_PI, 146.854/180.0*M_PI}};

    // std::vector<float> offsetJoints2 = {-0.05, 0.13089969389, 0.08726646259, -0.08726646259, 0.0, 0.0};

    // std::vector<std::pair<float, float> > minMaxValues3 = { {-109.376/180.0*M_PI, 106.492/180.0*M_PI},   
    //                                                         {-111.490/180.0*M_PI, 107.986/180.0*M_PI},
    //                                                         {-104.869/180.0*M_PI, 113.775/180.0*M_PI},
    //                                                         {-117.883/180.0*M_PI, 98.495/180.0*M_PI},
    //                                                         {-135.267/180.0*M_PI, 142.044/180.0*M_PI},
    //                                                         {-156.874/180.0*M_PI, 139.401/180.0*M_PI},
    //                                                         {-112.737/180.0*M_PI, 100.442/180.0*M_PI},   
    //                                                         {-106.202/180.0*M_PI, 110.798/180.0*M_PI},
    //                                                         {-103.401/180.0*M_PI, 116.820/180.0*M_PI},
    //                                                         {-105.499/180.0*M_PI, 110.352/180.0*M_PI},
    //                                                         {-130.213/180.0*M_PI, 142.940/180.0*M_PI},
    //                                                         {-153.010/180.0*M_PI, 146.854/180.0*M_PI}};

    // std::vector<float> offsetJoints3 = {-0.05, 0.087266463, 0.087266463, 0.0, 0.08726646259, 0.0, -0.05, 0.13089969389, 0.08726646259, -0.08726646259, 0.0, 0.0};

    // servoDriver =  new SCServo(serialPort);
    // if(!servoDriver->isConnected()){
    //     return -1;
    // }

    // std::vector<double> timeWrite;
    // std::vector<double> timeRead;
    // std::vector<double> timeAll;

    // std::vector<std::vector<double>> targetJoints(4);
    // std::vector<std::vector<double>> currentJoints(4);

    // rgbd::Graph2d plotTimes("times");
    // rgbd::Graph2d plotJoints("Joints");

    // float counter = 0;
    // float counter2 = 0;
    // float speedCounter = 1;
    // float speedCounter2 = 1;
    // for(;;){
    //     auto veryT0 = std::chrono::high_resolution_clock::now();
    //     counter +=speedCounter;
    //     counter2 +=speedCounter2*2;
    //     if(counter > 49){
    //         speedCounter = -1;
    //     }if(counter < -49){
    //         speedCounter = 1;
    //     }
    //     if(counter2 > 49){
    //         speedCounter2 = -1;
    //     }if(counter2< -49){
    //         speedCounter2 = 1;
    //     }
    //     // BOTH ARMS
    //     auto t4 = std::chrono::high_resolution_clock::now();
    //     std::vector<float> joints3 = {  0.0*M_PI/180.0, (counter/50)*45.0*M_PI/180.0, (counter2/50)*45.0*M_PI/180.0, 0.0*M_PI/180.0, 
    //                                     0.0*M_PI/180.0, (counter/50)*45.0*M_PI/180.0, (counter2/50)*45.0*M_PI/180.0, 0.0*M_PI/180.0};

    //     targetJoints[0].push_back(0.0*M_PI/180.0);
    //     targetJoints[1].push_back((counter/50)*45.0*M_PI/180.0);
    //     targetJoints[2].push_back((counter2/50)*45.0*M_PI/180.0);
    //     targetJoints[3].push_back(0.0*M_PI/180.0);
        

    //     unsigned char idn3 = joints3.size();
    //     unsigned char id3[idn3];
    //     unsigned short pos3[idn3], tim3[idn3], spee3[idn3];
    //     int cont1 = 0, cont2 = 0;
    //     for(unsigned k = 0; k < joints3.size(); k++){
    //         if(k < 4){
    //             id3[k] = 10 + cont1 + 1;
    //             pos3[k] = mapAngleToVal(minMaxValues3[k].first, minMaxValues3[k].second, joints3[k] + offsetJoints3[k]);
    //             tim3[k] = 500;
    //             spee3[k] = 0;
    //             cont1++;
    //         }else{
    //             id3[k] = 20 + cont2 + 1;
    //             pos3[k] = mapAngleToVal(minMaxValues3[k].first, minMaxValues3[k].second, joints3[k] + offsetJoints3[k]);
    //             tim3[k] = 500;
    //             spee3[k] = 0;
    //             cont2++;
    //         }
            
    //     }
    //     guard.lock(); 
    //     servoDriver->SyncWritePos(id3, idn3, pos3, tim3, spee3);
    //     guard.unlock();
    //     auto t5 = std::chrono::high_resolution_clock::now();
    //     float duration3 = std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count();
    //     std::cout << "Time send joints both ARMS: " << std::to_string(duration3) << std::endl;
    //     timeWrite.push_back(duration3);
    //     std::cout << "Position sended both ARMS: " << std::endl;
    //     for(unsigned k = 0; k < idn3; k++){
    //         std::cout << "  " << std::to_string(id3[k]) << "  " << pos3[k] << std::endl;
    //     }
    //     std::cout << std::endl;

    //     // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //     //////////////////////////////////////////////////
    //     // READ BOTH ARMS
    //     auto t10 = std::chrono::high_resolution_clock::now();
    //     std::vector<float> joints_read3;
    //     int cont3 = 0, cont4 = 0;
    //     for(unsigned n = 0; n < 8; n++){
    //         if(n < 4){
    //             guard.lock();
    //             int val = servoDriver->ReadPos(10 + cont3 + 1);
    //             guard.unlock();
    //             if(val < 0 || val > 1024){
    //                 joints_read3.push_back(-1);
    //             }else{
    //                 joints_read3.push_back( mapValToAngle( minMaxValues1[cont3].first, 
    //                                                 minMaxValues1[cont3].second,
    //                                                 val) - offsetJoints1[cont3]);
    //             }
    //             cont3++;
    //         }else{
    //             guard.lock();
    //             int val = servoDriver->ReadPos(20 + cont4 + 1);
    //             guard.unlock();
    //             if(val < 0 || val > 1024){
    //                 joints_read3.push_back(-1);
    //             }else{
    //                 joints_read3.push_back( mapValToAngle( minMaxValues2[cont4].first, 
    //                                                 minMaxValues2[cont4].second,
    //                                                 val) - offsetJoints2[cont4]);
    //             }
    //             cont4++;
    //         }    
    //     }

    //     currentJoints[0].push_back(joints_read3[0]);
    //     currentJoints[1].push_back(joints_read3[1]);
    //     currentJoints[2].push_back(joints_read3[2]);
    //     currentJoints[3].push_back(joints_read3[3]);

    //     auto t11 = std::chrono::high_resolution_clock::now();
    //     float duration6 = std::chrono::duration_cast<std::chrono::microseconds>(t11-t10).count();
    //     std::cout << "Time read joints both ARMS: " << std::to_string(duration6) << std::endl;
    //     timeRead.push_back(duration6);
    //     std::cout << "Read both ARMS: " << std::endl;
    //     for(unsigned n = 0; n < 8; n++){
    //         std::cout << "  " << joints_read3[n] << std::endl;
    //     }
    //     std::cout << std::endl;
    //     // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //     auto veryT1 = std::chrono::high_resolution_clock::now();
    //     float wholeDuration = std::chrono::duration_cast<std::chrono::microseconds>(veryT1-veryT0).count();
    //     timeAll.push_back(wholeDuration);

    //     int len = timeRead.size() < 100? timeRead.size():100;

    //     auto allSubset = std::vector<double>(timeAll.end()-len, timeAll.end());
    //     auto writeSubset = std::vector<double>(timeWrite.end()-len, timeWrite.end());
    //     auto readSubset = std::vector<double>(timeRead.end()-len, timeRead.end());

    //     plotTimes.clean();
    //     plotTimes.draw(allSubset,     255,100,100, rgbd::Graph2d::eDrawType::Lines);
    //     plotTimes.draw(writeSubset,   100,255,100, rgbd::Graph2d::eDrawType::Lines);
    //     plotTimes.draw(readSubset,    100,100,255, rgbd::Graph2d::eDrawType::Lines);
    //     plotTimes.show();
        
    //     plotJoints.clean();
    //     plotJoints.draw(std::vector<double>(targetJoints[0].end()-len, targetJoints[0].end()), 255,100,100, rgbd::Graph2d::eDrawType::Circles);
    //     plotJoints.draw(std::vector<double>(targetJoints[1].end()-len, targetJoints[1].end()), 255,255,100, rgbd::Graph2d::eDrawType::Circles);
    //     plotJoints.draw(std::vector<double>(targetJoints[2].end()-len, targetJoints[2].end()), 255,100,255, rgbd::Graph2d::eDrawType::Circles);
    //     plotJoints.draw(std::vector<double>(targetJoints[3].end()-len, targetJoints[3].end()), 100,100,255, rgbd::Graph2d::eDrawType::Circles);

    //     plotJoints.draw(std::vector<double>(currentJoints[0].end()-len, currentJoints[0].end()), 255,100,100, rgbd::Graph2d::eDrawType::Lines);
    //     plotJoints.draw(std::vector<double>(currentJoints[1].end()-len, currentJoints[1].end()), 255,255,100, rgbd::Graph2d::eDrawType::Lines);
    //     plotJoints.draw(std::vector<double>(currentJoints[2].end()-len, currentJoints[2].end()), 255,100,255, rgbd::Graph2d::eDrawType::Lines);
    //     plotJoints.draw(std::vector<double>(currentJoints[3].end()-len, currentJoints[3].end()), 100,100,255, rgbd::Graph2d::eDrawType::Lines);
    //     plotJoints.show();

    //     cv::waitKey(10);





    }
    return 0;
}
