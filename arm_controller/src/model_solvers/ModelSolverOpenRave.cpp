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

#include <arm_controller/model_solvers/ModelSolverOpenRave.h>

using namespace OpenRAVE;

namespace hecatonquiros{

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::init(const ModelSolver::Config &_config){
        if(initSingleton()){
            #ifdef HAS_OPENRAVE
                mConfig = _config;
                // lock the environment to prevent changes
                EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
                auto robot = mEnvironment->ReadRobotXMLFile(mConfig.robotFile);
                robot->SetName(mConfig.robotName);
                mEnvironment->Add(robot);
                return true;
            #endif
        }else{
            return false;
        }
    }


    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverOpenRave::joints(const std::vector<float> &_joints){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::vector<dReal> joints;
            for(auto &j:_joints){
                joints.push_back(j);
            }
            robot->SetDOFValues(joints); 
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    std::vector<float> ModelSolverOpenRave::joints() const{
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::vector< dReal > values;
            robot->GetDOFValues(values);

            std::vector<float> joints;
            for(auto &q:values){
                joints.push_back(q);
            }

            return joints;
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return std::vector<float>();    
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverOpenRave::jointsTransform(std::vector<Eigen::Matrix4f> &_transforms){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::vector<OpenRAVE::Transform> or_transforms;
            robot->GetBodyTransformations(or_transforms);


            for(unsigned k = 0; k < or_transforms.size(); k++){
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();;

                Eigen::Quaternionf q(or_transforms[k].rot.w, or_transforms[k].rot.x, or_transforms[k].rot.y, or_transforms[k].rot.z);   
                T.block<3,3>(0,0) = q.matrix();
                T(0,3) = or_transforms[k].trans.x;
                T(1,3) = or_transforms[k].trans.y;
                T(2,3) = or_transforms[k].trans.z;
                _transforms.push_back(T);
            }
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return Eigen::Matrix4f::Identity();    
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f ModelSolverOpenRave::jointTransform(int _idx){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::vector<OpenRAVE::Transform> or_transforms;
            robot->GetBodyTransformations(or_transforms);	
        
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();;

            Eigen::Quaternionf q(or_transforms[_idx].rot.w, or_transforms[_idx].rot.x, or_transforms[_idx].rot.y, or_transforms[_idx].rot.z);   
            T.block<3,3>(0,0) = q.matrix();
            T(0,3) = or_transforms[_idx].trans.x;
            T(1,3) = or_transforms[_idx].trans.y;
            T(2,3) = or_transforms[_idx].trans.z;

            return T;
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return Eigen::Matrix4f::Identity();    
        #endif 
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, bool _forceOri){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            OpenRAVE::ModuleBasePtr pikfast = OpenRAVE::RaveCreateModule(mEnvironment,"ikfast");
            mEnvironment->Add(pikfast,true,"");

            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::stringstream ssin,ssout;
            ssin << "LoadIKFastSolver " << robot->GetName() << " " << "Translation3D";
            // get the active manipulator
            OpenRAVE::RobotBase::ManipulatorPtr pmanip = robot->SetActiveManipulator(mConfig.manipulatorName);

            // Request solver
            try{
                if( !pikfast->SendCommand(ssout,ssin) ) {
                    RAVELOG_ERROR("failed to load iksolver\n");
                    return false;
                }
            }catch(openrave_exception &_e){
                std::cout <<_e.what() << std::endl;
                return false;
            }

            std::cout << "Getting ready for computing IK" << std::endl;

            Eigen::Quaternionf q(_pose.block<3,3>(0,0));

            Transform trans;
            trans.rot.x = q.x();
            trans.rot.y = q.y();
            trans.rot.z = q.z();
            trans.rot.w = q.w();
            trans.trans.x = _pose(0,3);
            trans.trans.y = _pose(1,3);
            trans.trans.z = _pose(2,3);

            std::cout << trans << std::endl;

            std::vector<dReal> vsolution;
            if( pmanip->FindIKSolution(OpenRAVE::IkParameterization(trans, IKP_Translation3D),vsolution,IKFO_IgnoreSelfCollisions) ) {
                std::cout << "FOUND SOLUTION" << std::endl;
                _joints.resize(vsolution.size());
                for(size_t i = 0; i < vsolution.size(); ++i) {
                    _joints[i] = vsolution[i];
                }
                return true;
            }
            else {
                std::cout << "NOT FOUND SOLUTION" << std::endl;
                // could fail due to collisions, etc
                return false;
            }
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return false;    
        #endif 
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, bool _forceOri){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            OpenRAVE::ModuleBasePtr pikfast = OpenRAVE::RaveCreateModule(mEnvironment,"ikfast");
            mEnvironment->Add(pikfast,true,"");

            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::stringstream ssin,ssout;
            ssin << "LoadIKFastSolver " << robot->GetName() << " " << "Translation3D";
            // get the active manipulator
            OpenRAVE::RobotBase::ManipulatorPtr pmanip = robot->SetActiveManipulator(mConfig.manipulatorName);

            // Request solver
            try{
                if( !pikfast->SendCommand(ssout,ssin) ) {
                    RAVELOG_ERROR("failed to load iksolver\n");
                    return false;
                }
            }catch(openrave_exception &_e){
                std::cout <<_e.what() << std::endl;
                return false;
            }

            std::cout << "Getting ready for computing IK" << std::endl;

            Eigen::Quaternionf q(_pose.block<3,3>(0,0));

            Transform trans;
            trans.rot.x = q.x();
            trans.rot.y = q.y();
            trans.rot.z = q.z();
            trans.rot.w = q.w();
            trans.trans.x = _pose(0,3);
            trans.trans.y = _pose(1,3);
            trans.trans.z = _pose(2,3);

            std::cout << trans << std::endl;

            std::vector<std::vector<dReal>> vsolutions;
            if( pmanip->FindIKSolutions(OpenRAVE::IkParameterization(trans, IKP_Translation3D),vsolutions,IKFO_IgnoreSelfCollisions) ) {
                std::cout << "FOUND SOLUTION" << std::endl;
                _joints.resize(vsolutions.size());
                for(size_t i = 0; i < vsolutions.size(); ++i) {
                    _joints[i].resize(vsolutions[i].size());
                    for(unsigned j = 0; j < vsolutions[i].size(); j++){
                        _joints[i][j] = vsolutions[i][j];
                    }
                }
                return true;
            }
            else {
                std::cout << "NOT FOUND SOLUTION" << std::endl;
                // could fail due to collisions, etc
                return false;
            }
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return false;    
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::initSingleton(){
        #ifdef HAS_OPENRAVE
            if(mInstance == nullptr){
                mInstance = new ModelSolverOpenRave();

                OpenRAVE::RaveInitialize(true);
                mEnvironment = OpenRAVE::RaveCreateEnvironment();
                mViewerThread = std::thread([&] {
                    {
                        EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
                        mEnvironment->SetDebugLevel(OpenRAVE::Level_Debug);
                        mViewer.reset();
                        mViewer = OpenRAVE::RaveCreateViewer(mEnvironment, "qtcoin");
                        if(!!mViewer){
                            mEnvironment->Add(mViewer);          
                        }
                    }
                    mViewer->main(true);
                });
            }
            return true;
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return false;    
        #endif 
    }

    ModelSolverOpenRave             *ModelSolverOpenRave::mInstance     = nullptr;
    OpenRAVE::EnvironmentBasePtr    ModelSolverOpenRave::mEnvironment  = nullptr;
    OpenRAVE::ViewerBasePtr         ModelSolverOpenRave::mViewer       = nullptr;
    std::thread                     ModelSolverOpenRave::mViewerThread;
}