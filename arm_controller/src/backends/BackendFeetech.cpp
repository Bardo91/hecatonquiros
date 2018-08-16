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
#include <hecatonquiros/backends/BackendFeetech.h>
#include <tinyxml2.h>

#include <iostream>
#include <chrono>

namespace hecatonquiros{
    std::mutex BackendFeetech::mComGuard;

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::init(const Config &_config){
        if(_config.configXML != ""){
            //std::cout << "Path Config XML " <<  _config.configXML << std::endl;
            bool result = extractDataXML(_config.configXML);
            if(result == false){
                return false;
            }

        }else{
            changeValues(_config.valuesMinMax);
            mOffsetJoints = _config.jointsOffsets;
        }
        
        mSerialPort = _config.port;
        mArmId = _config.armId;
        mServoDriver =  new SCServo(mSerialPort);
        if(mServoDriver->isConnected()){
            //mLoadChecker = std::thread([&](){
            //    while(mServoDriver->isConnected()){
            //        //for(auto &id:mUsedJoints){
            //        //    int load = mServoDriver->ReadLoadH(mArmId*10 + id + 1);
            //        //    if(load > 200)
            //        //        std::cout << "WARNING: Load of servo " << mArmId*10 + id + 1 << " is " << load <<std::endl;
            //        //    // 666 TODO do something with it;
            //        //}
            //        std::this_thread::sleep_for(std::chrono::milliseconds(200));
            //    }
            //});
            return true;
        }else{
            return false;
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::pose(const Eigen::Matrix4f &_pose, bool _blocking){
        return false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::joints(const std::vector<float> &_joints, bool _blocking){
        if(_joints.size() > mOffsetJoints.size()){
            for(int i = mOffsetJoints.size() ; i<_joints.size(); i++){
                mOffsetJoints.push_back(0); // Fill with zeros
            }
        }
        if(_joints.size() > mUsedJoints.size()){
            for(int i = mUsedJoints.size() ; i<_joints.size(); i++){
                mUsedJoints.push_back(i);
            }
        }
        
        mComGuard.lock();
        //if(mServoDriver->isConnected()){
            // unsigned char idn = _joints.size();
            // unsigned char id[idn];
            // unsigned short pos[idn], tim[idn], spee[idn];
            // for(unsigned i = 0; i < _joints.size(); i++){
		    //     id[i] = mArmId*10 + i + 1;
		    //     pos[i] = mapAngleToVal(mMinMaxValues[i].first, mMinMaxValues[i].second, _joints[i] + mOffsetJoints[i]);
		    //     tim[i] = mSpeed;
		    //     spee[i] = 0;
            // }
            // mServoDriver->SyncWritePos(id, idn, pos, tim, spee);
            // return true;
        //}
        for(unsigned i = 0; i < _joints.size(); i++){
            mServoDriver->WritePos(mArmId*10 + i + 1, mapAngleToVal(mMinMaxValues[i].first, mMinMaxValues[i].second, _joints[i] + mOffsetJoints[i]), mSpeed);
        }
        mComGuard.unlock();

        
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::claw(const int _action){
        if(_action == 0){
            mComGuard.lock();
            mServoDriver->WritePos(mArmId*10 + 7, 300, mSpeed);
            mComGuard.unlock();
            return true;
        }else if(_action == 1){
            mComGuard.lock();
            int pos = mServoDriver->ReadPos(mArmId*10 + 7);
            mServoDriver->WritePos(mArmId*10 + 7, pos, mSpeed);
            mComGuard.unlock();
            return true;
        
        }else if(_action == 2){
            mComGuard.lock();
            mServoDriver->WritePos(mArmId*10 + 7, 1023, mSpeed);
            mComGuard.unlock();
            return true;
        }else{
            std::cout << "Unrecognized command!" << std::endl;
            return false;
        }
        
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::changeValues(std::vector<std::pair<float, float> >_newvalues){
        for(unsigned i = 0; i < _newvalues.size(); i++){
            mMinMaxValues[i].first = _newvalues[i].first/180.0*M_PI;
            mMinMaxValues[i].second = _newvalues[i].second/180.0*M_PI;
        }
        return true;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetech::jointPos(const int _id){
        mComGuard.lock();
        int pos = mServoDriver->ReadPos(mArmId*10 + _id + 1);
        mComGuard.unlock();
        return pos;
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetech::jointLoad(const int _id){
        mComGuard.lock();
        int load = mServoDriver->ReadLoadH(mArmId*10 + _id + 1);
        mComGuard.unlock();
        return load;
    }

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<float> BackendFeetech::joints(int nJoints){
        std::vector<float> joints;
        std::vector<int> vals;
        mComGuard.lock();
        for(unsigned i = 0; i < nJoints; i++){
            int val = mServoDriver->ReadPos(mArmId*10 + i + 1);
            if(val < 0 || val > 1023){
                std::cout << "Reading error from feetech backend" << std::endl;
                return {};
            }
            vals.push_back(val);
            joints.push_back(mapValToAngle( mMinMaxValues[i].first, 
                                            mMinMaxValues[i].second,
                                            val - mOffsetJoints[i]));
        }
        mComGuard.unlock();
        printf("%d,%d,%d,%d,%d,%d\n", vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);

        return joints;
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool BackendFeetech::extractDataXML(std::string _pathXML){
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
        
        std::cout << "MinMaxValues: " << std::endl;
        for(int i = 0; i < extractedValues.size(); i++){
            std::cout << extractedValues[i].first << " | " << extractedValues[i].second << std::endl;
        }
        std::cout << "Offsets: " << std::endl;
        for(int i = 0; i < mOffsetJoints.size(); i++){
            std::cout << mOffsetJoints[i] << std::endl;
        }

        return true;
        
    }

    //-----------------------------------------------------------------------------------------------------------------
    int BackendFeetech::mapAngleToVal(float _minAngle, float _maxAngle, float _angle){
        return (_angle-_minAngle)/(_maxAngle-_minAngle)*(1023-0);
    }

        //-----------------------------------------------------------------------------------------------------------------
    float BackendFeetech::mapValToAngle(float _minAngle, float _maxAngle, int _val){
        return (_val-0.0)/(1023.0-0.0)*(_maxAngle-_minAngle) + _minAngle;
    }
}