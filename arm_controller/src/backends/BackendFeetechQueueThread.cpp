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


#include <serial/serial.h> 
#include <hecatonquiros/backends/BackendFeetechQueueThread.h>
#include <tinyxml2.h>

#include <iostream>
#include <chrono>

namespace hecatonquiros{
    std::mutex BackendFeetechQueueThread::mQueueGuard;

    bool BackendFeetechQueueThread::mActivateStatic = false;

    std::queue<std::shared_ptr<BackendFeetechQueueThread::DataQueue>> BackendFeetechQueueThread::mComQueue;
    std::thread BackendFeetechQueueThread::mQueueThread;

    SCServo *BackendFeetechQueueThread::mServoDriver;

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::init(const Config &_config){
        if(_config.configXML != ""){
            if(!extractDataXML(_config.configXML)) return false;
        }else{
            if(!changeValues(_config.valuesMinMax)) return false;
            mOffsetJoints = _config.jointsOffsets;
        }
        
        mSerialPort = _config.port;
        mArmId = _config.armId;

        return initStatic(mSerialPort);

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::initStatic(std::string _serialPort){
        
        if(!mActivateStatic){
            mActivateStatic = true;
            mQueueThread = std::thread(&BackendFeetechQueueThread::queueThread);
            mServoDriver =  new SCServo(_serialPort);
            return mServoDriver->isConnected();
        }
        
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    void BackendFeetechQueueThread::queueThread(){

        while(true){

            if(!mComQueue.empty()){
                auto data = mComQueue.front();
                //if(checkReq(data)){
                    switch(data->type){
                        case eTypeQ::Write:
                            {   
                                mServoDriver->SyncWritePos(data->idJoints, data->idn, data->pos, data->time, data->speed);
                                if(data->block){
                                    data->condVar.notify_one();
                                }
                                break;
                            }
                        case eTypeQ::Read:
                            {   
                                for(unsigned i = 0; i < data->idn; i++){
                                    data->valueRead.push_back(mServoDriver->ReadPos(data->idJoints[i]));
                                }
                                if(data->block){
                                    data->condVar.notify_one();
                                }
                                break;
                            }
                        case eTypeQ::Claw:
                            {   
                                mServoDriver->WritePos(data->idJoints[0], data->pos[0], data->time[0], data->speed[0]); 
                                if(data->block){
                                    data->condVar.notify_one();
                                }
                                break;
                            }
                    }
                //}
                mQueueGuard.lock();
                mComQueue.pop();
                mQueueGuard.unlock();
            }



        }
        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::checkReq(DataQueue _req){

        // 666 TODO: FILL THIS FUNCTION TO CHECK REQUEST


        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::joints(std::vector<float> &_joints, bool _blocking){
        
        unsigned char idn = _joints.size();
        unsigned char id[idn];
        unsigned short pos[idn], tim[idn], spee[idn];
        for(unsigned i = 0; i < _joints.size(); i++){
            id[i] = mArmId*10 + i + 1;
            pos[i] = mapAngleToVal(mMinMaxValues[i].first, mMinMaxValues[i].second, _joints[i] + mOffsetJoints[i]);
            tim[i] = mSpeed;
            spee[i] = 0;
        }

        std::shared_ptr<DataQueue> data(new DataQueue);
        data->type = eTypeQ::Write;
        data->idJoints = id;
        data->idn = idn;
        data->pos = pos;
        data->time = tim;
        data->speed = spee;
        data->block = _blocking;

        if(_blocking){
            mQueueGuard.lock();
            mComQueue.push(data);
            mQueueGuard.unlock();

            std::unique_lock<std::mutex> lck(data->guard);
            data->condVar.wait(lck);
        }else{
            mQueueGuard.lock();
            mComQueue.push(data);
            mQueueGuard.unlock();
        }   

        return true;

    }

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<float> BackendFeetechQueueThread::joints(int _nJoints, bool _blocking){

        std::vector<float> joints;
        unsigned char idn = _nJoints;
        unsigned char id[idn];
        for(unsigned i = 0; i < idn; i++){
            id[i] = mArmId*10 + i + 1;
        }

        std::shared_ptr<DataQueue> data(new DataQueue);
        data->type = eTypeQ::Read;
        data->idJoints = id;
        data->idn = idn;
        data->block = _blocking;
        
        if(_blocking){
            mQueueGuard.lock();
            mComQueue.push(data);
            mQueueGuard.unlock();

            std::unique_lock<std::mutex> lck(data->guard);
            data->condVar.wait(lck);
        }else{
            mQueueGuard.lock();
            mComQueue.push(data);
            mQueueGuard.unlock();
        }

        for(unsigned j = 0; j < idn; j++){
            int val = data->valueRead[j];
            if(val < 0 || val > 1024){
                std::cout << "Reading error from feetech backend. ID: " << id[j] << " val: " << val << std::endl;
                return {};
                //joints.push_back(0);
            }else{
                joints.push_back( mapValToAngle( mMinMaxValues[j].first, 
                                                mMinMaxValues[j].second,
                                                val) - mOffsetJoints[j]);
            }
        }
        
        return joints;

    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::claw(const int _action, bool _blocking){
        unsigned char id[1];
        unsigned short pos[1], speed[1], tim[1];
        if(_action == 0){
            id[0] = mArmId*10 + 7;
            pos[0] = 350;
            tim[0] = mSpeed;
            speed[0] = 0;
        }else if(_action == 1){
            // 666 TODO: NOT WORKING WITH REQUEST
        }else if(_action == 2){
            id[0] = mArmId*10 + 7;
            pos[0] = 1023;
            tim[0] = mSpeed;
            speed[0] = 0;
        }else{
            std::cout << "Unrecognized command!" << std::endl;
            return false;
        }

        std::shared_ptr<DataQueue> data(new DataQueue);
        data->type = eTypeQ::Claw;
        data->idJoints = id;
        data->pos = pos;
        data->time = tim;
        data->speed = speed;
        data->block = _blocking;
        if(_blocking){
            mQueueGuard.lock();
            mComQueue.push(data);
            mQueueGuard.unlock();

            std::unique_lock<std::mutex> lck(data->guard);
            data->condVar.wait(lck);
        }else{
            mQueueGuard.lock();
            mComQueue.push(data);
            mQueueGuard.unlock();
        }  

        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::pose(const Eigen::Matrix4f &_pose, bool _blocking){
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetechQueueThread::jointPos(const int _id){
        return -1;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetechQueueThread::jointLoad(const int _id){
        return -1;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetechQueueThread::jointTorque(const int _id, const bool _enable){
        return -1;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::changeValues(std::vector<std::pair<float, float> >_newvalues){
        for(unsigned i = 0; i < _newvalues.size(); i++){
            mMinMaxValues[i].first = _newvalues[i].first/180.0*M_PI;
            mMinMaxValues[i].second = _newvalues[i].second/180.0*M_PI;
        }
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetechQueueThread::extractDataXML(std::string _pathXML){
        tinyxml2::XMLDocument xml_doc;
        //std::cout << "XML Path: " << _pathXML << std::endl; 
        tinyxml2::XMLError resultXML = xml_doc.LoadFile(_pathXML.c_str());
        if(resultXML != tinyxml2::XML_SUCCESS){ 
            std::cout << "Error loading file" << std::endl; 
            std::cout << resultXML << std::endl; 
            return false;
        }

        std::vector<std::pair<float, float> > extractedValues;
        std::vector<float> extractedOffsets;

        tinyxml2::XMLNode* rootConfig = xml_doc.FirstChildElement("Config");
        if(rootConfig == nullptr){
            std::cout << "Error NO Config root" << std::endl; 
            return false;
        }
        else{   
            for (tinyxml2::XMLElement* childValues = rootConfig->FirstChildElement("MinMaxValues"); childValues != NULL; childValues = childValues->NextSiblingElement("MinMaxValues")){         
                for(int i = 1; i < 7; i++ ){
                    std::string childSearch = "Joint" + std::to_string(i) + "E1";
                    tinyxml2::XMLElement* itemValues = childValues->FirstChildElement(childSearch.c_str());
                    std::string sValueJointE1;
                    sValueJointE1 = itemValues->GetText();
                    float valueJointE1;
                    std::stringstream ssValueJointE1;
                    ssValueJointE1 << sValueJointE1; ssValueJointE1 >> valueJointE1;

                    childSearch = "Joint" + std::to_string(i) + "E2";
                    itemValues = childValues->FirstChildElement(childSearch.c_str());
                    std::string sValueJointE2;
                    sValueJointE2 = itemValues->GetText();
                    float valueJointE2;
                    std::stringstream ssValueJointE2;
                    ssValueJointE2 << sValueJointE2; ssValueJointE2 >> valueJointE2;
            
                    extractedValues.push_back(std::make_pair( valueJointE1, valueJointE2));
                }
            }
            for (tinyxml2::XMLElement* childOffsets = rootConfig->FirstChildElement("Offsets"); childOffsets != NULL; childOffsets = childOffsets->NextSiblingElement("Offsets")){  
                for(int i = 1; i < 7; i++ ){
                    std::string childSearch = "Joint" + std::to_string(i);
                    tinyxml2::XMLElement* itemOffsets = childOffsets->FirstChildElement(childSearch.c_str());

                    std::string sValueJoint;
                    sValueJoint = itemOffsets->GetText();
                    float valueJoint;
                    std::stringstream ssValueJoint;
                    ssValueJoint << sValueJoint; ssValueJoint >> valueJoint;

                    extractedOffsets.push_back(valueJoint);
                }   
            }
        }

        changeValues(extractedValues);
        mOffsetJoints = extractedOffsets;
        
        //std::cout << "MinMaxValues: " << std::endl;
        //for(int i = 0; i < extractedValues.size(); i++){
        //    std::cout << extractedValues[i].first << " | " << extractedValues[i].second << std::endl;
        //}
        //std::cout << "Offsets: " << std::endl;
        //for(int i = 0; i < mOffsetJoints.size(); i++){
        //    std::cout << mOffsetJoints[i] << std::endl;
        //}

        return true;
        
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetechQueueThread::mapAngleToVal(float _minAngle, float _maxAngle, float _angle){
        return (_angle-_minAngle)/(_maxAngle-_minAngle)*(1023-0);
    }

        //-----------------------------------------------------------------------------------------------------------------
    float BackendFeetechQueueThread::mapValToAngle(float _minAngle, float _maxAngle, int _val){
        return (_val-0.0)/(1023.0-0.0)*(_maxAngle-_minAngle) + _minAngle;
    }
}