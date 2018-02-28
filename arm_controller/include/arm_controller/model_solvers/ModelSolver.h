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


#ifndef HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVER_H_
#define HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVER_H_

#include <vector>
#include <Eigen/Eigen>

namespace hecatonquiros{
    class ModelSolver{
    public:
        /// Configuration class for Model solvers
        struct Config{
            enum class eType {Simple4DoF, OpenRave};
            eType type;
            std::string robotName;
            std::string manipulatorName;
        };

        static ModelSolver* create(const Config &_config);

        /// Set joints of robot
        /// \param _joints: desired joints
        virtual void joints(const std::vector<float> &_joints) = 0;
        
        /// Get current joints of robot
        virtual std::vector<float> joints() const = 0;

        /// Get transforms of joints
        virtual void jointsTransform(std::vector<Eigen::Matrix4f> &_transforms) = 0;

        /// Get transforms of specific joint
        virtual Eigen::Matrix4f jointTransform(int _idx) = 0;

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, bool _forceOri = true) = 0;


        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: list of possible solutions joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, bool _forceOri = true) = 0;
    };

}

#endif