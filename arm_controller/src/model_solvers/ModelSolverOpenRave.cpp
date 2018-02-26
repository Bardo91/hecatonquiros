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
    bool ModelSolverOpenRave::init(){
        if(initSingleton()){
            #ifdef HAS_OPENRAVE
                // lock the environment to prevent changes
                EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
                std::string robotName = "/home/bardo91/programming/catkin_positioner/src/PositionerEndTool/hecatonquiros/arm_controller/config/arm_gripper_5dof.robot.xml";
                auto robot = mEnvironment->ReadRobotXMLFile(robotName);
                robot->SetName("arm_1"); // 666 TODO customize names
                mEnvironment->Add(robot);
                return true;
            #endif
        }else{
            return false;
        }
    }


    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverOpenRave::joints(const std::vector<float> &_joints){
        EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
        std::vector<OpenRAVE::RobotBasePtr> robots;
        auto robot = mEnvironment->GetRobot("arm_1");

        std::vector<dReal> joints;
        for(auto &j:_joints){
            joints.push_back(j);
        }
        robot->SetDOFValues(joints); 
    }


    //-----------------------------------------------------------------------------------------------------------------
    std::vector<float> ModelSolverOpenRave::joints() const{
        EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
        std::vector<OpenRAVE::RobotBasePtr> robots;
        auto robot = mEnvironment->GetRobot("arm_1");

        std::vector< dReal > values;
        robot->GetDOFValues(values);

        std::vector<float> joints;
        for(auto &q:values){
            joints.push_back(q);
        }

        return joints;
    }


    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverOpenRave::jointsTransform(std::vector<Eigen::Matrix4f> &_transforms){
        EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
        std::vector<OpenRAVE::RobotBasePtr> robots;
        auto robot = mEnvironment->GetRobot("arm_1");

        std::vector<OpenRAVE::Transform> or_transforms;
        robot->GetBodyTransformations(or_transforms);


        for(unsigned k = 0; k < or_transforms.size(); k++){
            Eigen::Matrix4f T;

            Eigen::Quaternionf q(or_transforms[k].rot.w, or_transforms[k].rot.x, or_transforms[k].rot.y, or_transforms[k].rot.z);   
            T.block<3,3>(0,0) = q.matrix();
            T(0,3) = or_transforms[k].trans.x;
            T(1,3) = or_transforms[k].trans.y;
            T(2,3) = or_transforms[k].trans.z;
            _transforms.push_back(T);
        }
    }


    //-----------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f ModelSolverOpenRave::jointTransform(int _idx){
        EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
        std::vector<OpenRAVE::RobotBasePtr> robots;
        auto robot = mEnvironment->GetRobot("arm_1");

        std::vector<OpenRAVE::Transform> or_transforms;
        robot->GetBodyTransformations(or_transforms);	
    
        Eigen::Matrix4f T;

        Eigen::Quaternionf q(or_transforms[_idx].rot.w, or_transforms[_idx].rot.x, or_transforms[_idx].rot.y, or_transforms[_idx].rot.z);   
        T.block<3,3>(0,0) = q.matrix();
        T(0,3) = or_transforms[_idx].trans.x;
        T(1,3) = or_transforms[_idx].trans.y;
        T(2,3) = or_transforms[_idx].trans.z;

        return T;
    }


    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, bool _forceOri){

    }


    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::initSingleton(){
        #ifdef HAS_OPENRAVE
            if(mInstance == nullptr){
                mInstance = new ModelSolverOpenRave();

                OpenRAVE::RaveInitialize(true);
                mEnvironment = OpenRAVE::RaveCreateEnvironment();
                {
                    EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
                    mEnvironment->SetDebugLevel(OpenRAVE::Level_Debug);
                    // mViewer.reset();
                    // mViewer = OpenRAVE::RaveCreateViewer(mEnvironment, "qtcoin");
                    // if(!!mViewer){
                    //     mEnvironment->Add(mViewer);          
                    // }
                }
            }
            return true;
        #else
            std::cout << "OpenRAVE not installed, cannot instantiate ModelSolverOpenRAVE" << std::endl;
            return false;    
        #endif 
    }

    ModelSolverOpenRave             *ModelSolverOpenRave::mInstance     = nullptr;
    OpenRAVE::EnvironmentBasePtr    ModelSolverOpenRave::mEnvironment  = nullptr;
    OpenRAVE::ViewerBasePtr         ModelSolverOpenRave::mViewer       = nullptr;
}