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


#ifndef HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKENDFEETECH_H_
#define HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKENDFEETECH_H_

#include <hecatonquiros/backends/Backend.h>  
#include <hecatonquiros/backends/dep/SCServo.h>  

#include <thread>

namespace hecatonquiros{
    class BackendFeetech: public Backend {
    public:
        /// Default constructor. 
        BackendFeetech():Backend(){}

        /// This method is not implemented in arduino backend, it sends false by default.
        virtual bool pose(const Eigen::Matrix4f &_pose, bool _blocking = false);

        /// This method move the joints of the arm to the desired angle.
        /// \param _joints: vector containing the joints
        /// \param _blocking: set blocking or not blocking operation
        /// \return true if joints are send or set without errors, false if something failed.
        virtual bool joints(const std::vector<float> &_joints, bool _blocking = false);

        /// Method for actuating to claws if implemented and attached
        /// \param _action: 0 close, 1 stop, 2 open;
        virtual bool claw(const int _action);
    private:
        // Initialize communication with the arduino with the given configuration.
        // \param _config: Configuration file. The port name and the arm id are the only parameters needed.
        // \return true. 
        virtual bool init(const Config &_config);

        // Map from 0 to 1023 according to min and max angle
        int mapAngleToVal(float _minAngle, float _maxAngle, float _angle);

        /// Change MinMaxValues
        /// \param _newvalues: vector containing the MinMaxValues
        virtual bool changeValues(std::vector<std::pair<float, float> > _newvalues);
    private:
        std::string mSerialPort;
        int mArmId;

        SCServo *mServoDriver;
        float mSpeed = 500;     // 666 SEE HOW TO TUNE SPEED
        std::vector<std::pair<float, float> > mMinMaxValues = { {-110.0/180.0*M_PI, 110/180.0*M_PI},    // 666 SEE HOW TO TUNE FOREACH ROBOT
                                                                {-110.0/180.0*M_PI, 110/180.0*M_PI},
                                                                {-110.0/180.0*M_PI, 115/180.0*M_PI},
                                                                {-135.0/180.0*M_PI, 135/180.0*M_PI},
                                                                {-135.0/180.0*M_PI, 135/180.0*M_PI},
                                                                {-135.0/180.0*M_PI, 135/180.0*M_PI}};
        
        std::vector<float> mOffsets = {};   /// 666 TODO: do it

        std::thread mLoadChecker;
        std::vector<int> mUsedJoints;
        std::vector<float> mOffsetJoints;
    };
}

#endif