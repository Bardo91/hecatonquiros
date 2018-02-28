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


#ifndef HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVERSIMPLE4DOF_H_
#define HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVERSIMPLE4DOF_H_

#include <arm_controller/model_solvers/ModelSolver.h>

namespace hecatonquiros{
    class ModelSolverSimple4Dof: public ModelSolver{
    public:
        /// Set joints of robot
        /// \param _joints: desired joints
        void joints(const std::vector<float> &_joints);
        
        /// Get current joints of robot
        std::vector<float> joints() const;
        
        /// Get transforms of joints
        void jointsTransform(std::vector<Eigen::Matrix4f> &_transforms);

        /// Get transforms of specific joint
        Eigen::Matrix4f jointTransform(int _idx);

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        bool checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, bool _forceOri = true);

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: list of possible solutions joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        bool checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, bool _forceOri = true){ return false;}

    private:
        void updateTransforms();

    private:
        std::vector<float> mJoints = std::vector<float>(4);
        bool mUpdatedTransforms = false;

        std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign>> mJointsTransform;
        Eigen::Matrix<float,4,4,Eigen::DontAlign> mFinalT;

        float mHumerus = 0.15;
        float mRadius = 0.09;
        float mBaseHeight = 0.08;


    };

}

#endif