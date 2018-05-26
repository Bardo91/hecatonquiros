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


#ifndef HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVEROPENRAVE_H_
#define HECATONQUIROS_ARMCONTROLLER_MODELSOLVERS_MODELSOLVEROPENRAVE_H_

#include <hecatonquiros/model_solvers/ModelSolver.h>

#ifdef HAS_OPENRAVE
    #include <openrave-core.h>
    #include <openrave/planningutils.h>
#endif

#include <string>
#include <thread>
#include <vector>
#include <iostream>

namespace hecatonquiros{
    class ModelSolverOpenRave: public ModelSolver{
    public:

        /// Set joints of robot
        /// \param _joints: desired joints
        virtual void joints(const std::vector<float> &_joints);
        
        /// Get current joints of robot
        virtual std::vector<float> joints() const;
        
        /// Get transforms of joints
        virtual void jointsTransform(std::vector<Eigen::Matrix4f> &_transforms);

        /// Get transforms of specific joint
        virtual Eigen::Matrix4f jointTransform(int _idx);

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, IK_TYPE _type = IK_TYPE::IK_3D);

        /// Check if exists IK for a given pose
        /// \param _pose: desired pose
        /// \param _joints: list of possible solutions joints for given pose
        /// \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
        virtual bool checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, IK_TYPE _type = IK_TYPE::IK_3D);

        /// Five end effector pose given joints without moving the arm
        /// \param _joints: list of possible solutions joints for given pose
        virtual Eigen::Matrix4f testIk(const std::vector<float> &_joints);

        /// Get the points of the desired trajectory
        /// \param _pose: desired points that the trajectory must have
        /// \param _traj: list of possible solutions joints for given poses
        /// \param _time: total time of the trajectory
        virtual bool getPointsTrajectory(std::vector<Eigen::Matrix4f> _pose, std::vector<std::vector<double>> &_traj, float &_time);

        virtual Eigen::MatrixXf jacobian();

        virtual Eigen::MatrixXf rotationJacobian();

    #ifdef HAS_OPENRAVE
        OpenRAVE::RobotBasePtr robot();
    #endif
    
    #ifdef HAS_OPENRAVE
    public: // Specific interface for OpenRave access
        /// Get copy of environment
        static OpenRAVE::EnvironmentBasePtr cloneEnvironment();
        static OpenRAVE::EnvironmentBasePtr getEnvironment();
        static bool addObject(std::string _xmlObject, std::string _name = "object");
        static void moveObject(Eigen::Matrix4f _T, std::string _name);
        static void setTransparencyObject(std::string _name, float _val);

        /// Draw a line on the current environment. Keep returned object to keep the line
        /// \param _init: beginning of the line
        /// \param _end: end of the line
        /// \param _r: from 0 to 1
        /// \param _g: from 0 to 1
        /// \param _b: from 0 to 1
        /// \param _a: from 0 to 1 
        static OpenRAVE::GraphHandlePtr drawLine(Eigen::Vector3f _init, Eigen::Vector3f _end, float _width = 0.01, float  _r = 1,float  _g = 0,float  _b = 0,float _a = 1);

    #endif 
    protected:
        virtual bool init(const ModelSolver::Config &_config);

        ModelSolver::IK_TYPE checkIfType(ModelSolver::IK_TYPE _type);

    private:
        // Singleton interface
        static bool initSingleton(bool _enableVis);
        static ModelSolverOpenRave *mInstance;
        #ifdef HAS_OPENRAVE
            static OpenRAVE::EnvironmentBasePtr mEnvironment;
            static OpenRAVE::ViewerBasePtr      mViewer;
            static std::thread                  mViewerThread;
            static OpenRAVE::ModuleBasePtr      mIkFast;
        #endif

    private:
        #ifdef HAS_OPENRAVE
            std::vector<OpenRAVE::GraphHandlePtr> poses;
            OpenRAVE::GraphHandlePtr mPoseManipX;
            OpenRAVE::GraphHandlePtr mPoseManipY;
            OpenRAVE::GraphHandlePtr mPoseManipZ;
            OpenRAVE::GraphHandlePtr mPoseIK;
        #endif
    };

}

#endif
