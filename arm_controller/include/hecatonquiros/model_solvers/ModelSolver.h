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
            enum class eType {Simple4DoF, OpenRave, Ros};
            eType type;
            std::string robotName;
            std::string manipulatorName;
            std::string robotFile;
            std::vector<float> offset = {0,0,0};    /// {x, y, z}
            std::vector<float> rotation = {0,0,0,1};     /// {x, y, z,w}
            bool visualizer = false;
            std::string environment;
        };

        enum class IK_TYPE {IK_3D, IK_5D, IK_6D};

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
        /// \param _pose: desired pose. If 5DoF, the Z axis is used as target direction.
        /// \param _joints: joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, IK_TYPE _type = IK_TYPE::IK_3D) = 0;

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose. If 5DoF, the Z axis is used as target direction.
        /// \param _joints: list of possible solutions joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, IK_TYPE _type = IK_TYPE::IK_3D) = 0;

        /// Five end effector pose given joints without moving the arm
        /// \param _joints: list of possible solutions joints for given pose
        virtual Eigen::Matrix4f testIk(const std::vector<float> &_joints) = 0;

        /// Get the points of the desired trajectory
        /// \param _pose: desired points that the trajectory must have
        /// \param _traj: list of possible solutions joints for given poses
        /// \param _time: total time of the trajectory
        virtual bool getPointsTrajectory(std::vector<Eigen::Matrix4f> _pose, std::vector<std::vector<double>> &_traj, float &_time) = 0;

    protected:
        ModelSolver(){};
        virtual bool init(const ModelSolver::Config &_config) = 0;

        Config mConfig;
    };

}

#endif
