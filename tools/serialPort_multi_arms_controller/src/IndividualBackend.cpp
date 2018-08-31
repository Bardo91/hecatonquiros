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

#include "IndividualBackend.h"

#include <iostream>

bool IndividualBackend::init(std::string _json, int _id){
    hecatonquiros::Backend::Config bc; 

    if(mConfigFile.Parse(_json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    std::string sEnableBc = "enable_backend"+std::to_string(_id);
    const char *cEnableBc = sEnableBc.c_str();
    bool enableBackend = mConfigFile[cEnableBc].GetBool();

    if(enableBackend){ 
        std::string serialPort = mConfigFile["serial_port"].GetString();
        std::string sConfigFile = "configXML"+std::to_string(_id);
        const char *cConfigFile = sConfigFile.c_str();
        bc.configXML = mConfigFile[cConfigFile].GetString(); 
		bc.type = hecatonquiros::Backend::Config::eType::Feetech; 
        bc.port = serialPort;
        bc.armId = _id;
    }else{
        bc.type = hecatonquiros::Backend::Config::eType::Dummy;
    }

    mBackend = hecatonquiros::Backend::create(bc);

    return true;
}

bool IndividualBackend::joints(std::vector<float> &_joints, bool _actuateBackend){
    return mBackend->joints(_joints,_actuateBackend);
}

std::vector<float> IndividualBackend::joints(int _nJoints){
    return mBackend->joints(_nJoints);
}

bool IndividualBackend::claw(int _action){
    return mBackend->claw(_action);
}

int IndividualBackend::jointPos(int _id){
    return mBackend->jointPos(_id);
}

int IndividualBackend::jointLoad(int _id){
    return mBackend->jointLoad(_id);
}
