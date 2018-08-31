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

#ifndef INDIVIDUALARM_H_
#define INDIVIDUALARM_H_


#include <hecatonquiros/Arm4DoF.h>
#include <string>
#include <vector>
#include <memory>

#include "rapidjson/document.h"

/// Class that encapsulate generic arm
class IndividualArm{
public:
    /// Intialize class with two equal arms.
    /// \param _robotFile: path to openrave robot description.
    /// \param _enableBackend: set bc to dummy (false) or feetech (true).
    /// \return true if all good .
    bool init(std::string _json);

    /// Set current joints of arm.
    /// \param _arm: arm selected.
    /// \param _joints: vector with joints values.
    void joints(std::vector<float> &_joints, bool _actuateBackend = false);

    /// get current pose of arm. 
    /// \param _arm: arm selected.
    Eigen::Matrix4f pose();

    /// Return current joints of arm.
    /// \param _arm: arm selected.
    std::vector<float> joints() const;

    /// Pose in cartesian coordinates (meters).
    bool checkIk(Eigen::Matrix4f _pose, std::vector<float> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D);
    
    /// Pose in cartesian coordinates (meters).
    bool checkIk(Eigen::Matrix4f _pose, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D);
    
    /// Pose in cartesian coordinates (meters).
    std::vector<bool> checkIks(std::vector<std::shared_ptr<Eigen::Matrix4f>> _pose, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D);

    /// Grasp given object
    

    hecatonquiros::Arm4DoF *mArm;
private:
    rapidjson::Document mConfigFile;
};

#endif
