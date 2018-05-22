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

#include <hecatonquiros/model_solvers/ModelSolverOpenRave.h>

using namespace OpenRAVE;

namespace hecatonquiros{

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::init(const ModelSolver::Config &_config){
        if(initSingleton(_config.visualizer)){
            #ifdef HAS_OPENRAVE
                mConfig = _config;
                // lock the environment to prevent changes
                EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
                if(mConfig.robotFile == ""){
                    if(mConfig.environment != "")
                        mEnvironment->Load(_config.environment);

                    auto robot = mEnvironment->GetRobot(mConfig.robotName);
                    if(!robot)
                        return false;
                }else{
                    auto robot = mEnvironment->ReadRobotXMLFile(mConfig.robotFile);
                    robot->SetName(mConfig.robotName);
                    OpenRAVE::Transform offset(OpenRAVE::Vector({mConfig.rotation[0], mConfig.rotation[1], mConfig.rotation[2], mConfig.rotation[3]}),OpenRAVE::Vector({mConfig.offset[0], mConfig.offset[1], mConfig.offset[2]}));
                    robot->SetTransform(offset);
                    mEnvironment->Add(robot);
                }

                return true;
            #endif
        }else{
            return false;
        }
    }


    //-----------------------------------------------------------------------------------------------------------------
    ModelSolver::IK_TYPE ModelSolverOpenRave::checkIfType(ModelSolver::IK_TYPE _type){
        EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
        std::vector<OpenRAVE::RobotBasePtr> robots;
        auto robot = mEnvironment->GetRobot(mConfig.robotName);
        int nDof = robot->GetDOF();
        
        hecatonquiros::ModelSolver::IK_TYPE type;
        switch(nDof){
            case 3:
                if(_type == 3 || _type == 31)
                    type = _type;
                else{
                    type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;
                    //std::cout << "Sending IK of type "<<  _type << " but n dof is " << nDof << " using Ik type " << type << std::endl; 
                }

                break;
            case 4:
                if(_type == 3 || _type == 31)
                    type = _type;
                else{
                    type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;
                    //std::cout << "Sending IK of type "<<  _type << " but n dof is " << nDof << " using Ik type " << type << std::endl; 
                }
                break;
            case 5:
                if(_type <= 5 || _type == 31)
                    type = _type;
                else{
                    type = hecatonquiros::ModelSolver::IK_TYPE::IK_5D;
                    //std::cout << "Sending IK of type "<<  _type << " but n dof is " << nDof << " using Ik type " << type << std::endl; 
                }
                break;
            case 6:
                if(_type <= 6 || _type == 31)
                    type = _type;
                else{
                    type = hecatonquiros::ModelSolver::IK_TYPE::IK_6D;
                    //std::cout << "Sending IK of type "<<  _type << " but n dof is " << nDof << " using Ik type " << type << std::endl; 
                }
                break;
        }
        return type;
    }


    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverOpenRave::joints(const std::vector<float> &_joints){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);


            std::vector< dReal > currentJoints;
            robot->GetDOFValues(currentJoints);

            std::vector<dReal> joints(robot->GetDOF ());
            std::vector<int> indices(robot->GetDOF ());
            for(unsigned i = 0; i < robot->GetDOF (); i++){
                if(i < _joints.size()){
                    joints[i] = _joints[i];
                }else{
                    joints[i] = currentJoints[i];
                }
                indices[i] = i;
            }
            robot->SetDOFValues(joints, 1, indices);
            
            poses.clear();

            std::vector<Eigen::Matrix4f> transforms;
            jointsTransform(transforms);
            
            //for(auto &pose:transforms){
            //    RaveVector<float> p1 = {pose(0,3),pose(1,3), pose(2,3)}, p2;
            //    RaveVector<float> dir = {pose(0,0),pose(1,0), pose(2,0)};
            //    p2 = p1 + dir*0.05;
            //    poses.push_back(mEnvironment->drawarrow(p1, p2,0.001,OpenRAVE::RaveVector< float >(1, 0, 0, 1)));
            //    dir = {pose(0,1),pose(1,1), pose(2,1)};
            //    p2 = p1 + dir*0.05;
            //    poses.push_back(mEnvironment->drawarrow(p1, p2,0.001,OpenRAVE::RaveVector< float >(0, 1, 0, 1)));
            //    dir = {pose(0,2),pose(1,2), pose(2,2)};
            //    p2 = p1 + dir*0.05;
            //    poses.push_back(mEnvironment->drawarrow(p1, p2,0.001,OpenRAVE::RaveVector< float >(0, 0, 1, 1)));
            //}
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
        #endif 
    }


    //-----------------------------------------------------------------------------------------------------------------
    std::vector<float> ModelSolverOpenRave::joints() const{
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
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
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::vector<OpenRAVE::Transform> or_transforms;
            robot->GetLinkTransformations (or_transforms);

            for(auto &link: robot->GetLinks()){
                auto orTransform = link->GetTransform();
                OpenRAVE::TransformMatrix orMatrix(orTransform);

                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();;

                T(0,0) = orMatrix.m[0];     T(0,1) = orMatrix.m[1];     T(0,2) = orMatrix.m[2];     T(0,3) = orMatrix.trans.x;
                T(1,0) = orMatrix.m[4];     T(1,1) = orMatrix.m[5];     T(1,2) = orMatrix.m[6];     T(1,3) = orMatrix.trans.y;
                T(2,0) = orMatrix.m[8];     T(2,1) = orMatrix.m[9];     T(2,2) = orMatrix.m[10];    T(2,3) = orMatrix.trans.z;

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
            robot->GetLinkTransformations (or_transforms);	
        
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();;

            Eigen::Quaternionf q;
            q.x() = or_transforms[_idx].rot.x;
            q.y() = or_transforms[_idx].rot.y;
            q.z() = or_transforms[_idx].rot.z;
            q.w() = or_transforms[_idx].rot.w; 
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
    bool ModelSolverOpenRave::checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, IK_TYPE _type){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            _type = checkIfType(_type);

            std::string stringType;
            OpenRAVE::IkParameterizationType intType;
            if(_type == IK_TYPE::IK_3D){
                stringType = "Translation3D";
                intType = IKP_Translation3D;
            }else if (_type == IK_TYPE::IK_5D){
                stringType = "TranslationDirection5D";
                intType = IKP_TranslationDirection5D;
            }else if (_type == IK_TYPE::IK_6D){
                stringType = "Transform6D";
                intType = IKP_Transform6D;
            }else if (_type == IK_TYPE::LOOK){
                stringType = "Lookat3D";
                intType = IKP_Lookat3D;
            }else{
                assert(false);
            }

            std::stringstream ssin,ssout;
            ssin << "LoadIKFastSolver " << robot->GetName() << " " << stringType;
            // get the active manipulator
            OpenRAVE::RobotBase::ManipulatorPtr pmanip = robot->SetActiveManipulator(mConfig.manipulatorName);

            // Request solver
            try{
                if( !mIkFast->SendCommand(ssout,ssin) ) {
                    RAVELOG_ERROR("failed to load iksolver\n");
                    return false;
                }
            }catch(openrave_exception &_e){
                std::cout <<_e.what() << std::endl;
                return false;
            }

            OpenRAVE::IkParameterization ikParam;
            if(_type == IK_TYPE::IK_3D){
                ikParam.SetTranslation3D({_pose(0,3), _pose(1,3), _pose(2,3)});
            }else if (_type == IK_TYPE::IK_5D){
                OpenRAVE::RAY ray;
                ray.pos = {_pose(0,3), _pose(1,3), _pose(2,3)};
                ray.dir = {_pose(0,2), _pose(1,2), _pose(2,2)};
                ikParam.SetTranslationDirection5D(ray);

                RaveVector<float> p1 = ray.pos, p2;
                RaveVector<float> dir = ray.dir;
                p2 = p1 + dir*0.05;
                //mPoseManipZ = mEnvironment->drawarrow(p1, p2,0.005,OpenRAVE::RaveVector< float >(0, 0, 1, 1));
            }else if (_type == IK_TYPE::IK_6D){
                RaveTransformMatrix<float> matT;
                
                matT.rotfrommat(_pose(0,0), _pose(0,1), _pose(0,2),
                                _pose(1,0), _pose(1,1), _pose(1,2),
                                _pose(2,0), _pose(2,1), _pose(2,2));
                
                matT.trans.x = matT.m[3] = _pose(0,3);
                matT.trans.y = matT.m[7] = _pose(1,3);
                matT.trans.z = matT.m[11] = _pose(2,3);

                OpenRAVE::Transform T(matT);
                ikParam.SetTransform6D(T);
            }else if(_type == IK_TYPE::LOOK){
                ikParam.SetLookat3D({_pose(0,3), _pose(1,3), _pose(2,3)});
            }else{
                assert(false);
            }

            std::vector<dReal> vsolution;
            if( pmanip->FindIKSolution(ikParam,vsolution,IKFO_IgnoreSelfCollisions) ) {
                _joints.resize(vsolution.size());
                for(size_t i = 0; i < vsolution.size(); ++i) {
                    _joints[i] = vsolution[i];
                }
                return true;
            }
            else {
                return false;
            }
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return false;    
        #endif 
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::checkIk(const Eigen::Matrix4f &_pose, std::vector<std::vector<float>> &_joints, IK_TYPE _type){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            _type = checkIfType(_type);

            std::string stringType;
            OpenRAVE::IkParameterizationType intType;
            if(_type == IK_TYPE::IK_3D){
                stringType = "Translation3D";
                intType = IKP_Translation3D;
            }else if (_type == IK_TYPE::IK_5D){
                stringType = "TranslationDirection5D";
                intType = IKP_TranslationDirection5D;
            }else if (_type == IK_TYPE::IK_6D){
                stringType = "Transform6D";
                intType = IKP_Transform6D;
            }else{
                assert(false);
            }

            std::stringstream ssin,ssout;
            ssin << "LoadIKFastSolver " << robot->GetName() << " " << stringType;
            // get the active manipulator
            OpenRAVE::RobotBase::ManipulatorPtr pmanip = robot->SetActiveManipulator(mConfig.manipulatorName);

            // Request solver
            try{
                if( !mIkFast->SendCommand(ssout,ssin) ) {
                    RAVELOG_ERROR("failed to load iksolver\n");
                    return false;
                }
            }catch(openrave_exception &_e){
                std::cout <<_e.what() << std::endl;
                return false;
            }

            std::cout << "Getting ready for computing IK" << std::endl;

            OpenRAVE::IkParameterization ikParam;
            if(_type == IK_TYPE::IK_3D){
                ikParam.SetTranslation3D({_pose(0,3), _pose(1,3), _pose(2,3)});
            }else if (_type == IK_TYPE::IK_5D){
                OpenRAVE::RAY ray;
                ray.pos = {_pose(0,3), _pose(1,3), _pose(2,3)};
                ray.dir = {_pose(0,2), _pose(1,2), _pose(2,2)};
                ikParam.SetTranslationDirection5D(ray);

                RaveVector<float> p1 = ray.pos, p2;
                RaveVector<float> dir = ray.dir;
                p2 = p1 + dir*0.05;
                mPoseManipZ = mEnvironment->drawarrow(p1, p2,0.005,OpenRAVE::RaveVector< float >(0, 0, 1, 1));
            }else if (_type == IK_TYPE::IK_6D){
                RaveTransformMatrix<float> matT;
                
                matT.rotfrommat(_pose(0,0), _pose(0,1), _pose(0,2),
                                _pose(1,0), _pose(1,1), _pose(1,2),
                                _pose(2,0), _pose(2,1), _pose(2,2));
                
                matT.trans.x = matT.m[3] = _pose(0,3);
                matT.trans.y = matT.m[7] = _pose(1,3);
                matT.trans.z = matT.m[11] = _pose(2,3);

                OpenRAVE::Transform T(matT);
                ikParam.SetTransform6D(T);
            }else{
                assert(false);
            }

            std::vector<std::vector<dReal>> vsolutions;
            if( pmanip->FindIKSolutions(ikParam,vsolutions,IKFO_IgnoreSelfCollisions) ) {
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

    Eigen::Matrix4f ModelSolverOpenRave::testIk(const std::vector<float> &_joints){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
            std::vector<OpenRAVE::RobotBasePtr> robots;
            auto robot = mEnvironment->GetRobot(mConfig.robotName);

            std::vector<dReal> joints(robot->GetDOF ());
            std::vector<int> indices(robot->GetDOF ());
            for(unsigned i = 0; i < robot->GetDOF (); i++){
                joints[i] = _joints[i];
                indices[i] = i;
            }
            robot->SetDOFValues(joints, 1, indices);

            std::vector<OpenRAVE::Transform> or_transforms;
            robot->GetBodyTransformations(or_transforms);

            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();;

            auto orT = or_transforms.back();
            Eigen::Quaternionf q(orT.rot.w, orT.rot.x, orT.rot.y, orT.rot.z);   
            T.block<3,3>(0,0) = q.matrix();
            T(0,3) = orT.trans.x;
            T(1,3) = orT.trans.y;
            T(2,3) = orT.trans.z;


            //std::vector<Eigen::Matrix4f> transforms;
            //jointsTransform(transforms);
            //Eigen::Matrix4f pose = transforms.back();
            //RaveVector<float> p1 = {pose(0,3),pose(1,3), pose(2,3)}, p2;
            //RaveVector<float> dir = {pose(0,2),pose(1,2), pose(2,2)};
            //p2 = p1 + dir*0.05;
            //mPoseManipZ = mEnvironment->drawarrow(p1, p2,0.001,OpenRAVE::RaveVector< float >(0, 0, 1, 1));
            //dir = {pose(0,1),pose(1,1), pose(2,1)};
            //p2 = p1 + dir*0.05;
            //mPoseManipY = mEnvironment->drawarrow(p1, p2,0.001,OpenRAVE::RaveVector< float >(0, 1, 0, 1));
            //dir = {pose(0,0),pose(1,0), pose(2,0)};
            //p2 = p1 + dir*0.05;
            //mPoseManipX = mEnvironment->drawarrow(p1, p2,0.001,OpenRAVE::RaveVector< float >(1, 0, 0, 1));

            return T;
        #else
            return Eigen::Matrix4f::Identity();
        #endif

    }


    #ifdef HAS_OPENRAVE
        OpenRAVE::RobotBasePtr ModelSolverOpenRave::robot(){
            return mEnvironment->GetRobot(mConfig.robotName);
        }
    #endif

    #ifdef HAS_OPENRAVE
        //-----------------------------------------------------------------------------------------------------------------
        OpenRAVE::EnvironmentBasePtr ModelSolverOpenRave::cloneEnvironment(){
            return mEnvironment->CloneSelf(Clone_All);

        }

        //-----------------------------------------------------------------------------------------------------------------
        OpenRAVE::EnvironmentBasePtr ModelSolverOpenRave::getEnvironment(){
            return mEnvironment;
        }
    #endif

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::addObject(std::string _xmlObject, std::string _name){
        #ifdef HAS_OPENRAVE
            if(mInstance != nullptr){
                EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
                auto object = mEnvironment->ReadKinBodyXMLFile(_xmlObject);
                object->SetName(_name);
                mEnvironment->Add(object);
                return object != nullptr;
            }
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverOpenRave::moveObject(Eigen::Matrix4f _T, std::string _name){
        #ifdef HAS_OPENRAVE
            if(mInstance != nullptr){
                auto object = mEnvironment->GetKinBody(_name);
                RaveTransformMatrix<float> matT;
                for(unsigned i = 0; i < 3; i++){
                    for(unsigned j = 0; j < 3; j++){
                        matT.m[j*4 + i] = _T(j,i);
                    }
                }
                matT.trans.x = _T(0,3); matT.m[3] = _T(0,3);
                matT.trans.y = _T(1,3); matT.m[7] = _T(1,3);
                matT.trans.z = _T(2,3); matT.m[11] = _T(2,3);


                OpenRAVE::Transform T(matT);
                object->SetTransform(T);
            }
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverOpenRave::setTransparencyObject(std::string _name, float _val){
        #ifdef HAS_OPENRAVE
            auto object = mEnvironment->GetKinBody(_name);
            auto links = object->GetLinks();
            for(auto&link : links){
                for(auto &geo: link->GetGeometries()){
                    geo->SetTransparency(_val);
                }
            }	
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    OpenRAVE::GraphHandlePtr ModelSolverOpenRave::drawLine(Eigen::Vector3f _init, Eigen::Vector3f _end, float _width, float _r, float  _g, float  _b, float  _a){
        #ifdef HAS_OPENRAVE
            return mEnvironment->drawarrow	(	RaveVector< float >(_init[0], _init[1], _init[2], 1),
                                                RaveVector< float >(_end[0], _end[1], _end[2], 1),
                                                _width,
                                                RaveVector< float >(_r, _g, _b, _a)
                                                );
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::getPointsTrajectory(std::vector<Eigen::Matrix4f> _pose, std::vector<std::vector<double>> &_traj, float &_time){
        #ifdef HAS_OPENRAVE
            EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());

            auto robot = mEnvironment->GetRobot(mConfig.robotName);
            TrajectoryBasePtr trajectory = RaveCreateTrajectory(mEnvironment, "");
            trajectory->Init(robot->GetActiveConfigurationSpecification());

            hecatonquiros::ModelSolver::IK_TYPE type;
            type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;

            std::vector<std::vector<float>> joints;
            //joints.resize(_pose.size());
            for(int i = 0; i < _pose.size(); i++){
                std::vector<float> auxJoint;
                std::cout << "(getPointsTrajectory) Check Point: " << _pose[i](0,3) << " , " << _pose[i](1,3) << " , " << _pose[i](2,3) << std::endl;
                if(checkIk(_pose[i], auxJoint, type)){
                    joints.push_back(auxJoint);
                }else{
                    std::cout << "Not FOUND IK (getPointsTrajectory)" << std::endl;
                }
                
            } 

            for(int i = 0; i < robot->GetDOF (); i++){
                std::vector<OpenRAVE::dReal> auxJoints;
                for(auto &v:joints[i]){
                    auxJoints.push_back(v);
                }
                trajectory->Insert(i, auxJoints, false);
            }
            
            planningutils::SmoothActiveDOFTrajectory(trajectory, robot);

            for(int i = 0; i < trajectory->GetNumWaypoints(); i++){
                std::vector<double> traj;
                trajectory->GetWaypoints(i, i+1, traj);

                std::vector<double> auxTraj;
                for(int j = 0; j < 4; j++){
                    auxTraj.push_back(traj.at(j));

                }
                _traj.push_back( std::vector<double>() );
                _traj[i].swap(auxTraj);
            }



            _time = trajectory->GetDuration();

            return true;
        #endif
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverOpenRave::initSingleton(bool _enableVis){
        #ifdef HAS_OPENRAVE
            if(mInstance == nullptr){
                mInstance = new ModelSolverOpenRave();

                OpenRAVE::RaveInitialize(false, OpenRAVE::Level_Error);

                std::cout << "Load Plugins" << std::endl;

                OpenRAVE::RaveLoadPlugin("basecontrollers");
                OpenRAVE::RaveLoadPlugin("baserobots");
                OpenRAVE::RaveLoadPlugin("basesamplers");
                OpenRAVE::RaveLoadPlugin("basesensors");
                OpenRAVE::RaveLoadPlugin("bulletrave");
                OpenRAVE::RaveLoadPlugin("configurationcache");
                OpenRAVE::RaveLoadPlugin("dualmanipulation");
                OpenRAVE::RaveLoadPlugin("fclrave");
                OpenRAVE::RaveLoadPlugin("grasper");
                OpenRAVE::RaveLoadPlugin("ikfastsolvers");
                OpenRAVE::RaveLoadPlugin("logging");
                OpenRAVE::RaveLoadPlugin("mobyrave");
                OpenRAVE::RaveLoadPlugin("oderave");
                OpenRAVE::RaveLoadPlugin("pqprave");
                OpenRAVE::RaveLoadPlugin("rmanipulation");
                OpenRAVE::RaveLoadPlugin("rplanners");
                OpenRAVE::RaveLoadPlugin("textserver");
                if(_enableVis){
                    OpenRAVE::RaveLoadPlugin("qtosgrave");
                    OpenRAVE::RaveLoadPlugin("qtcoinrave");
                }

                mEnvironment = OpenRAVE::RaveCreateEnvironment();
                
                mIkFast = OpenRAVE::RaveCreateModule(mEnvironment,"ikfast");
                mEnvironment->Add(mIkFast,true,"");
                
                mViewerThread = std::thread([&](bool _enableVis) {
                    {
                        EnvironmentMutex::scoped_lock lock(mEnvironment->GetMutex());
                        if(_enableVis){
                            mViewer.reset();
                            mViewer = OpenRAVE::RaveCreateViewer(mEnvironment, "qtcoin");
                            if(!!mViewer){
                                mEnvironment->Add(mViewer);
                            }
                        }
                    }
                    if(_enableVis){
                        mViewer->main(true);
                    }
                }, _enableVis);
            }
            return true;
        #else
            std::cout << "OpenRAVE not installed, cannot use ModelSolverOpenRAVE" << std::endl;
            return false;    
        #endif 
    }

    ModelSolverOpenRave             *ModelSolverOpenRave::mInstance     = nullptr;
    #ifdef HAS_OPENRAVE
        OpenRAVE::EnvironmentBasePtr    ModelSolverOpenRave::mEnvironment  = nullptr;
        OpenRAVE::ViewerBasePtr         ModelSolverOpenRave::mViewer       = nullptr;
        OpenRAVE::ModuleBasePtr         ModelSolverOpenRave::mIkFast = nullptr;
    #endif
    std::thread                     ModelSolverOpenRave::mViewerThread;
}
