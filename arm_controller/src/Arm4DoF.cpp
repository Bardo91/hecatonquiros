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


#include <hecatonquiros/Arm4DoF.h>
#include <hecatonquiros/model_solvers/ModelSolver.h>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cassert>

namespace hecatonquiros{
    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const ModelSolver::Config &_modelConfig, const Backend::Config &_backendConfig) {
        mBackend = Backend::create(_backendConfig);
        mArmId = _backendConfig.armId;
        mModelSolver = ModelSolver::create(_modelConfig);
        openClaw();
    }

    //---------------------------------------------------------------------------------------------------------------------
    Arm4DoF::Arm4DoF(const Backend::Config &_backendConfig) {
        mBackend = Backend::create(_backendConfig);
        mArmId = _backendConfig.armId;
        mModelSolver = nullptr;
        openClaw();
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::home(){
        std::vector<float> angles = {mHome1, mHome2, mHome3, mHome4, mHome5, mHome6};
        joints(angles, true);
        openClaw();

        if(mModelSolver != nullptr){  /// 666 Just for visualizing transforms in joints in ModelSolverOpenRave
            std::vector<Eigen::Matrix4f> transforms;
            mModelSolver->jointsTransform(transforms);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::joints(std::vector<float> _q, bool _actuateBackend) {
        if(mModelSolver != nullptr){
            mModelSolver->joints(_q);
        }

        if(_actuateBackend){
            if(mBackend != nullptr){
                mBackend->joints(_q, false);
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    std::vector<float> Arm4DoF::joints() const {
        if(mBackend != nullptr){
            // 666 will not work without model solver
            return mBackend->joints(mModelSolver->joints().size()); 
        }else{
            if(mModelSolver != nullptr){
                return mModelSolver->joints();
            }else{
                std::cout << "No model solver instantiated" << std::endl;
                return {};
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::position(Eigen::Vector3f _position, float _wirst) {
        if(mModelSolver != nullptr){
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block<3,1>(0,3) = _position;
            std::vector<float> angles;
            if(mModelSolver->checkIk(pose, angles, ModelSolver::IK_TYPE::IK_3D)){
                angles[3] = _wirst; //
                joints(angles);
            }
        }else{
            std::cout << "No model solver instantiated" << std::endl;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Vector3f Arm4DoF::position() {
        if(mModelSolver != nullptr){
            std::vector<Eigen::Matrix4f> transforms;
            mModelSolver->jointsTransform(transforms);
            return transforms.back().block<3,1>(0,3);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return Eigen::Vector3f();
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Vector3f _position, std::vector<float> &_angles){
        if(mModelSolver != nullptr){
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block<3,1>(0,3) = _position;
            return mModelSolver->checkIk(pose, _angles, ModelSolver::IK_TYPE::IK_3D);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::getSmoothTraj(std::vector<Eigen::Matrix4f> _pose, std::vector<std::vector<double>> &_traj, float &_time){
        if(mModelSolver != nullptr){
            return mModelSolver->getPointsTrajectory(_pose, _traj, _time); 
            return false;
        }else{
            std::cout << "No model solver OPENRAVE instantiated" << std::endl;
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _pose, std::vector<float> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
        if(mModelSolver != nullptr){
            return mModelSolver->checkIk(_pose, _angles, _type);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return false;
        }
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::checkIk(Eigen::Matrix4f _pose, std::vector<std::vector<float>> &_angles, hecatonquiros::ModelSolver::IK_TYPE _type){
        if(mModelSolver != nullptr){
            return mModelSolver->checkIk(_pose, _angles, _type);
        }else{
            std::cout << "No model solver instantiated" << std::endl;
            return false;
        }
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f Arm4DoF::pose() const {
        if(mBackend != nullptr){
            // 666 will not work without model solver 666 causes glitch and not very good for multithreading...
            auto joints = mBackend->joints(mModelSolver->joints().size());
            Eigen::Matrix4f pose = mModelSolver->testFK(joints); 
            return pose;
        }else{
            if(mModelSolver != nullptr){
                std::vector<Eigen::Matrix4f> transforms;
                mModelSolver->jointsTransform(transforms);
                return transforms.back();
            }else{
                std::cout << "No model solver instantiated" << std::endl;
                return Eigen::Matrix4f::Identity();
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::closeClaw(){
        if(/*!mClawClosed && */mBackend != nullptr){
            mBackend->claw(0);
            mClawClosed = true;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::openClaw(){
        if(/*mClawClosed && */mBackend != nullptr){
            mBackend->claw(2);
            mClawClosed = false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    void Arm4DoF::stopClaw(){
        if(mBackend != nullptr){
            mBackend->claw(1);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::isInit() const{
        return mBackend != nullptr;
    }


    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::jacobianStep(Eigen::Vector3f &_position, std::vector<float> &_joints, float _alpha){
        if(mModelSolver != nullptr){
            Eigen::MatrixXf jacobian = mModelSolver->jacobian();

            Eigen::Vector3f errVec = _position - pose().block<3,1>(0,3);
            Eigen::MatrixXf I(jacobian.cols(), jacobian.cols());
            I.setIdentity();
            Eigen::VectorXf incJoints = 
                    (jacobian.transpose()*jacobian +I*0.1).inverse()*jacobian.transpose()*errVec;

            // std::cout << incJoints << std::endl;
            if(std::isnan(incJoints[0]))
                return true;

            _joints = joints();
            for(int i=0; i < _joints.size(); i++) _joints[i] += incJoints[i]*_alpha;
        }else{
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool Arm4DoF::jacobianStep(Eigen::Matrix4f &_pose, std::vector<float> &_joints, float _alphaPosition, float _alphaRotation){
        // Get current jacobians
        Eigen::MatrixXf positionJacobian = mModelSolver->jacobian();
        Eigen::MatrixXf rotationJacobian = mModelSolver->rotationJacobian();

        Eigen::MatrixXf jacobian(positionJacobian.rows()+rotationJacobian.rows(), positionJacobian.cols());
        jacobian << positionJacobian, rotationJacobian;


        // Compute the errors
        Eigen::Vector3f positionTarget = _pose.block<3,1>(0,3);
        Eigen::Quaternionf qTarget(_pose.block<3,3>(0,0));

        Eigen::VectorXf errVec(7);

        Eigen::Matrix4f currentPose = pose();
        Eigen::Quaternionf currentQuat((Eigen::Matrix3f)currentPose.block<3,3>(0,0));

        Eigen::Vector4f qdiff = { qTarget.w() - currentQuat.w(), qTarget.x() - currentQuat.x(), qTarget.y() - currentQuat.y(), qTarget.z() - currentQuat.z()};	
        Eigen::Vector4f qsum =  { qTarget.w() + currentQuat.w(), qTarget.x() + currentQuat.x(), qTarget.y() + currentQuat.y(), qTarget.z() + currentQuat.z()};

        Eigen::Vector4f errVecRot;						
        if(qdiff.norm() > qsum.norm()){	// improvement from http://openrave-users-list.185357.n3.nabble.com/Manipulator-CalculateRotationJacobian-td2873122.html#a2873146
            errVecRot =  {	-qTarget.x() - currentQuat.x(),
                            -qTarget.y() - currentQuat.y(),
                            -qTarget.z() - currentQuat.z(),
                            -qTarget.w() - currentQuat.w()};
        }else{
            errVecRot =  qdiff;
        }

        errVec <<  (positionTarget - currentPose.block<3,1>(0,3))*_alphaPosition, errVecRot*_alphaRotation;
        
        // Perform the step
        Eigen::MatrixXf I(jacobian.cols(), jacobian.cols());
        I.setIdentity();
        Eigen::VectorXf incJoints =  (jacobian.transpose()*jacobian +I*0.1).inverse()*jacobian.transpose()*errVec;
        // Eigen::MatrixXf pinv = pseudoinverse(jacobian);
        // Eigen::VectorXf incJoints = pinv*errVec;

        // Check if result is numerically stable
        if(std::isnan(incJoints[0]))
            return false;

        _joints = joints();
        for(int i=0; i < _joints.size(); i++) _joints[i] += incJoints[i];

        float accumIncs = 0;
        for(int i=0; i < _joints.size(); i++) accumIncs += fabs(incJoints[i]);
        if(accumIncs < 0.001){
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    int Arm4DoF::readPos(int _joint){
        if(mBackend != nullptr){
            return mBackend->jointPos(_joint);
        }else{
            return 0;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    int Arm4DoF::readLoad(int _joint){
        if(mBackend != nullptr){
            return mBackend->jointLoad(_joint);
        }else{
            return 0;
        }
    }
}
