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

#ifndef DUALMANIPULATOR_H_
#define DUALMANIPULATOR_H_


#include <hecatonquiros/Arm4DoF.h>
#include <string>
#include <vector>
#include <memory>

#include "rapidjson/document.h"

/// Class that encapsulate two arms
class DualManipulator{
public:
    /// Intialize class with two equal arms.
    /// \param _robotFile: path to openrave robot description.
    /// \param _enableBackend: set bc to dummy (false) or feetech (true).
    /// \return true if all good .
    bool init(std::string _json);

    enum class eArm {LEFT, RIGHT};

    /// Set current joints of selected arm.
    /// \param _arm: arm selected.
    /// \param _joints: vector with joints values.
    void joints(eArm _arm, std::vector<float> &_joints, bool _actuateBackend = false);

    /// get current pose of arm. 
    /// \param _arm: arm selected.
    Eigen::Matrix4f pose(eArm _arm);

    /// Return current joints of selected arm.
    /// \param _arm: arm selected.
    std::vector<float> joints(eArm _arm) const;

    /// Pose in cartesian coordinates (meters).
    bool checkIk(eArm _arm, Eigen::Matrix4f _pose, std::vector<float> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D);
    
    /// Pose in cartesian coordinates (meters).
    bool checkIk(eArm _arm, Eigen::Matrix4f _pose, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D);
    
    /// Pose in cartesian coordinates (meters).
    std::vector<bool> checkIks(eArm _arm, std::vector<std::shared_ptr<Eigen::Matrix4f>> _pose, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D);

    /// Grasp given object
    

    hecatonquiros::Arm4DoF *mLeftArm, *mRightArm;
private:
    rapidjson::Document mConfigFile;
};

#endif
