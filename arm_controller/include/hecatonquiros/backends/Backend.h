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


#ifndef HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKEND_H_
#define HECATONQUIROS_ARMCONTROLLER_BACKENDS_BACKEND_H_

#include <string>
#include <Eigen/Eigen>

namespace serial{class Serial;} // Forward declaration neede for config structure

namespace hecatonquiros{
    class Backend{
    public:
        struct Config{
            /// Type of backend
            enum class eType {Arduino, Gazebo, Feetech, Dummy};
            eType type;

            /// Shared Configuration
            int             armId;

            /// Config for Arduino and feetech
            std::string     port = "";

            /// Config specific for arduino
            int             baudrate = -1;
            serial::Serial *sharedSerialPort = nullptr;


            /// Config for gazebo
            std::string topic = "";
	    
            /// offsets  to joints
            std::vector<float> jointsOffsets;

            /// MinMaxValues for feetech servos
            std::vector<std::pair<float, float> > valuesMinMax;

            /// File directory configuration of MinMaxValues and Offsets
            std::string configXML = "";

        };

        static Backend* create(const Config &_config);

        /// \brief abstract method for sending arm to a desired pose
        virtual bool pose(const Eigen::Matrix4f &_pose, bool _blocking = false) = 0;

        /// \brief abstract method for moving joints of the arm to the desired angle
        virtual bool joints(const std::vector<float> &_joints, bool _blocking = false) = 0;

        /// \brief abstract method for actuating to claws if implemented and attached
        /// \param _action: 0 close, 1 stop, 2 open;
        virtual bool claw(const int _action) = 0;

        /// \brief Request for particular information of the hardware.
        /// Each backend might have a different behaviour.
        /// \param _cmd: command sent to the backend for requesting something
        virtual float request(std::string &_cmd) {return 0.0;}
    protected:
        Backend() {}  
        // \brief abstract method for initialization of the class
        virtual bool init(const Config &_config) = 0;

    };

    class BackendDummy: public Backend{
        virtual bool pose(const Eigen::Matrix4f &_pose, bool _blocking = false){return true;}
        virtual bool joints(const std::vector<float> &_joints, bool _blocking = false){return true;}
        virtual bool claw(const int _action){return true;}
    private:
        virtual bool init(const Config &_config){return true;}
    };
}

#endif
