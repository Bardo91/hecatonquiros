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


#include <hecatonquiros/Arm4DoF.h>
#define HAS_OPENRAVE
#include <hecatonquiros/model_solvers/ModelSolverOpenRave.h>
#include <Eigen/QR>
#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <serial/serial.h> 

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

//--------------------------------------------------------------------------------------------------------------------
void pointsInCircleNU(double _radius, Eigen::Vector3f _center, Eigen::Vector3f n, Eigen::Vector3f u, unsigned _nPoints, std::vector<Eigen::Vector3f> &_poses){
	Eigen::Vector3f nxu = u.cross(n);

	_poses.resize(_nPoints);

	double incT = 2 * M_PI / _nPoints;
	for (unsigned i = 0; i < _nPoints; i++) {
		_poses[i] = _radius*cos(i*incT)*u + _radius*sin(i*incT)*nxu + _center;
	}
}


int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "arm_tester");

	hecatonquiros::Backend::Config backendConfig;
	backendConfig.configXML = _argv[1];
	backendConfig.type = hecatonquiros::Backend::Config::eType::Feetech; 
	backendConfig.port = "/dev/ttyACM0"; 
	backendConfig.armId =2;

	srand(time(NULL));

    hecatonquiros::ModelSolver::Config modelSolverConfig;
    modelSolverConfig.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
	modelSolverConfig.robotName = "arm";
    modelSolverConfig.manipulatorName = "manipulator";
    modelSolverConfig.robotFile = _argv[2];
    modelSolverConfig.offset = {0.0,0.0,0.0};
	
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ())
		* Eigen::AngleAxisf(1*M_PI,  Eigen::Vector3f::UnitY());
	Eigen::Quaternionf q(m);
    
	modelSolverConfig.rotation = {q.w(), q.x(), q.y(), q.z()};
    modelSolverConfig.visualizer = true;

	hecatonquiros::Arm4DoF arm(modelSolverConfig, backendConfig);

	arm.home();
	
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	
	while(true){
		char c;
		while(std::cin >> c) {
			switch(c){
				case 'o':
					std::cout << "open" <<std::endl;
					arm.openClaw();
					break;
				case 'c':
					std::cout << "close" <<std::endl;
					arm.closeClaw();
					break;
				case 's':
					std::cout << "stop" <<std::endl;
					arm.stopClaw();
					break;
				case 'h':
					std::cout << "home" <<std::endl;
					arm.home();
					break;
				case 'S':
				{
					auto joints = arm.joints();
					for(auto &v: joints){
						std::cout << v << ", ";
					}
					std::cout << std::endl;
					break;
				}
				case 't':
				{
					bool finGetPoints = false;
					std::vector<std::vector<float>> WayPoints;
					while(!finGetPoints){
						float x, y, z;
						std::cout << "Enter new Point for trajectory" << std::endl;
						std::cout << "X: ";
						std::cin >> x;
						std::cout << "Y: ";
						std::cin >> y;
						std::cout << "Z: ";
						std::cin >> z;

						std::vector<float> xyz = {x, y, z};
						WayPoints.push_back(xyz);

						std::cout << "Add more points? 1 -> to break: ";
						bool fin;
						std::cin >> fin;
						if(fin){
							finGetPoints = true;
						}
					}

					std::vector<Eigen::Matrix4f> poses;
        			for(int i = 0; i < WayPoints.size(); i++){
        			    Eigen::Matrix4f pose;
        			    pose = Eigen::Matrix4f::Identity();
        			    pose(0,3) = WayPoints[i][0];
        			    pose(1,3) = WayPoints[i][1];
        			    pose(2,3) = WayPoints[i][2];
						std::cout << "Save in POSES: " << pose(0,3) << " , " << pose(1,3) << " , " << pose(2,3) << std::endl;
        			    poses.push_back(pose);
        			}

        			std::vector<std::vector<double>> jointsTraj;
        			float timeTraj;
        			if(arm.getSmoothTraj( poses, jointsTraj, timeTraj)){
						std::cout << "Time Trajectory: " << timeTraj << std::endl;
        			    for(int i = 0; i < jointsTraj.size(); i++){
        			        std::vector<float> auxJoints;
        			        for(auto &v:jointsTraj[i]){
        			            auxJoints.push_back(v);
        			        }
						
						std::cout << "auxJoints:"; 
						for(int i = 0; i < auxJoints.size(); i++){
							std::cout << "  " << auxJoints.at(i);
						}
						std::cout << std::endl;
                		arm.joints(auxJoints, true);

                		int timeToWait = (timeTraj/jointsTraj.size())*10000;
                		std::this_thread::sleep_for(std::chrono::milliseconds(timeToWait));
        			    }
        			}
        			else{
        			    std::cout << "Error in Smooth Trajectory" << std::endl;
        			}
					break;
				}
				case 'p':
				{
					float x, y, z;
					bool forceOri;
					std::cout << "position" <<std::endl;
					std::cout << "x: " <<std::endl;
					std::cin >> x;
					std::cout << "y: " <<std::endl;
					std::cin >> y;
					std::cout << "z: " <<std::endl;
					std::cin >> z;
					std::cout << "Force ori: " << std::endl;
					std::cin >> forceOri;
					Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
					pose(0,3) = x;
					pose(1,3) = y;
					pose(2,3) = z;
					
					hecatonquiros::ModelSolver::IK_TYPE type;
					if(forceOri){
						float dx,dy,dz;
						std::cout << "dx: " << std::endl;
						std::cin >> dx;
						std::cout << "dy: " << std::endl;
						std::cin >> dy;
						std::cout << "dz: " << std::endl;
						std::cin >> dz;
						Eigen::Vector3f zAxis = {dx, dy, dz};
						zAxis /=zAxis.norm();
						pose.block<3,1>(0,2) = zAxis;
						type = hecatonquiros::ModelSolver::IK_TYPE::IK_6D;

					}else{
						type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;
					}
					std::vector<float> joints;
					auto start = std::chrono::high_resolution_clock::now();
					if(arm.checkIk(pose, joints, type)){
						arm.joints(joints, true);
					}else{
						std::cout << "Not found IK" << std::endl;
					}
					auto end = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
					std::cout << duration.count() << std::endl;
					break;
				}
				case 'r':
				{
					std::vector<float> joints;
					auto pose = arm.pose();
					int counter = 0;
					int nDof = arm.joints().size();
					hecatonquiros::ModelSolver::IK_TYPE type;
					switch(nDof){
						case 3:
							type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;
							break;
						case 4:
							type = hecatonquiros::ModelSolver::IK_TYPE::IK_3D;
							break;
						case 5:
							type = hecatonquiros::ModelSolver::IK_TYPE::IK_5D;
							break;
						case 6:
							type = hecatonquiros::ModelSolver::IK_TYPE::IK_6D;
							break;
					}
					for(;;){
						counter++;
						Eigen::Vector3f incTrans = Eigen::MatrixXf::Random(3,1)*0.005;
						pose.block<3,1>(0,3) += incTrans;
						Eigen::Matrix3f m;
						m = Eigen::AngleAxisf	((double(rand())/RAND_MAX - 0.5)*0.1, 	Eigen::Vector3f::UnitX())
							* Eigen::AngleAxisf	((double(rand())/RAND_MAX - 0.5)*0.1,  	Eigen::Vector3f::UnitY())
							* Eigen::AngleAxisf	((double(rand())/RAND_MAX - 0.5)*0.1, 	Eigen::Vector3f::UnitZ());
						pose.block<3,3>(0,0) = m*pose.block<3,3>(0,0);
						if(arm.checkIk(pose, joints, type)){
							arm.joints(joints, true);
							std::cout << "Tries: " << counter << std::endl;
							std::cout << incTrans << std::endl;
							std::cout << m << std::endl;	
							std::cout << pose <<std::endl;
							break;
						}else{
							std::cout << "Not found IK" << std::endl;
						}
					}
					break;
				}
				case 'w':
				{
					float dx, dy, dz, distance, step_size, rot_speed;
					int delay;
					std::cout << "Moving aling a line rotating the pose. Pick direction, distance, step size and rotation speed." <<std::endl;
					std::cout << "dx: "; std::cin >> dx;
					std::cout << "dy: "; std::cin >> dy;
					std::cout << "dz: "; std::cin >> dz;
					std::cout << "distance (m): "; std::cin >> distance;
					std::cout << "step_size (m): "; std::cin >> step_size;
					std::cout << "rot_speed (deg): "; std::cin >> rot_speed;
					std::cout << "delay (ms): "; std::cin >> delay;
					if(step_size < 0 || rot_speed == 0){
						std::cout << "Bad step size or rot_speed" <<std::endl;
						continue;
					}
					Eigen::Vector3f dir = {dx, dy, dz};
					dir /=dir.norm();
					int iters = distance/step_size;
					Eigen::Matrix3f rotInc;
					rotInc = Eigen::AngleAxisf(rot_speed/180.0*M_PI, Eigen::Vector3f::UnitX());
					auto pose = arm.pose();
					for(unsigned i = 0; i < iters; i++){
						pose.block<3,1>(0,3) +=dir*step_size;
						pose.block<3,3>(0,0) = rotInc*pose.block<3,3>(0,0);
						std::vector<float> joints;
						std::this_thread::sleep_for(std::chrono::milliseconds(delay));
						if(arm.checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
							arm.joints(joints, true);
						}else{
							if(arm.checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
								arm.joints(joints, true);
							}else{
								std::cout << "Not found IK" << std::endl;
							}
						}
					}
					break;
				}
				case 'z':
				{
					float circle_radius;
					int delay, nPoints;
					std::cout << "Circle mode." <<std::endl;
					std::cout << "radius: "; std::cin >> circle_radius;
					std::cout << "points: "; std::cin >> circle_radius;
					std::cout << "delay (ms): "; std::cin >> delay;
					auto pose = arm.pose();
					std::vector<Eigen::Vector3f> circle_points;
					pointsInCircleNU(circle_radius, pose.block<3,1>(0,3), pose.block<3,1>(0,0), pose.block<3,1>(0,2), nPoints, circle_points);
					for(unsigned i = 0; i < nPoints; i++){
						pose.block<3,1>(0,3) = circle_points[i];
						std::vector<float>joints;
						if(arm.checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
							arm.joints(joints, true);
						}else{
							if(arm.checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
								arm.joints(joints, true);
							}else{
								std::cout << "Not found IK" << std::endl;
							}
						}
						std::this_thread::sleep_for(std::chrono::milliseconds(delay));
					}
					break;
				}
				case 'j':
				{	
					std::vector<float> joints = arm.joints();
					int n;
					std::cout << "id joint: " <<std::endl;
					std::cin >> n;
					float joint;
					std::cout << "val joint " << n << ": " <<std::endl;
					std::cin >> joint;
					
					joints[n] = joint/180.0*M_PI;
					arm.joints(joints, true);
					break;
				}
				case 'J':
				{	
					std::vector<float> joints;
					int n;
					std::cout << "joints" <<std::endl;
					std::cout << "n joints: " <<std::endl;
					std::cin >> n;
					for(unsigned i = 0; i < n; i++){
						float joint;
						std::cout << i << " joint: " <<std::endl;
						std::cin >> joint;
						joints.push_back(joint/180.0*M_PI);
					}
					arm.joints(joints, true);
					break;
				}
				case 'i':
				{
					auto pose = arm.pose();
					std::cout << pose << std::endl;
					break;
				}
				case 'g':
				{
					float x, y, z;
					std::cout << "position" <<std::endl;
					std::cout << "x: " <<std::endl;
					std::cin >> x;
					std::cout << "y: " <<std::endl;
					std::cin >> y;
					std::cout << "z: " <<std::endl;
					std::cin >> z;
					float step;
					std::cout << "step:  " << std::endl;
					std::cin >> step;

					Eigen::Vector3f position = {x,y,z};
					float error = 1;
					while(error > 0.005){
						std::vector<float> joints;
						arm.jacobianStep(position, joints, step);
						arm.joints(joints);
						Eigen::Vector3f errVec = position - arm.pose().block<3,1>(0,3);
						error = errVec.norm();
						std::this_thread::sleep_for(std::chrono::milliseconds(30));
					}
					break;
				}
				case 'G':
				{
					float step = 0.01;
					float error = 1;
					Eigen::Matrix3f targetRot;
					targetRot = Eigen::AngleAxisf(5*M_PI/6, Eigen::Vector3f::UnitY());
					Eigen::Quaternionf qTarget(targetRot); 

					while(error > 0.005){
						Eigen::MatrixXf jacobian = arm.modelSolver()->rotationJacobian();
						std::cout << jacobian << std::endl;
						
						Eigen::Quaternionf currentQuat((Eigen::Matrix3f)arm.pose().block<3,3>(0,0));

						Eigen::Quaternionf errQuat = currentQuat.inverse()*qTarget;
						Eigen::Vector4f errVec = {errQuat.x(), errQuat.y(), errQuat.z(), errQuat.w()};
						std::cout <<"errVec: " << errVec.transpose() << std::endl;
						Eigen::MatrixXf I(jacobian.cols(), jacobian.cols());
						I.setIdentity();
						Eigen::VectorXf incJoints = 
								(jacobian.transpose()*jacobian +I*0.1).inverse()*jacobian.transpose()*errVec;

						std::cout << "incJoinst" << incJoints.transpose() << std::endl;
						if(std::isnan(incJoints[0]))
							return true;

						std::vector<float> joints = arm.joints();
						for(int i=0; i < joints.size(); i++) joints[i] += incJoints[i]*step;
						arm.joints(joints);

						errQuat = Eigen::Quaternionf(arm.pose().block<3,3>(0,0)).inverse()*qTarget;
						errVec = {errQuat.x(), errQuat.y(), errQuat.z(), errQuat.w()};
						error = errVec.norm();

						std::this_thread::sleep_for(std::chrono::milliseconds(30));
						currentQuat = Eigen::Quaternionf((Eigen::Matrix3f)arm.pose().block<3,3>(0,0));
						Eigen::Vector4f currentVec = {currentQuat.x(), currentQuat.y(), currentQuat.z(), currentQuat.w()};
						std::cout << "targetvec: " << qTarget.x()<<", "<< qTarget.y()<<", "<< qTarget.z()<<", "<< qTarget.w()<<", " << std::endl;
						std::cout << "currentvect: " << currentVec.transpose() << std::endl;
						std::cout << "error: " << error << std::endl;
					}
					break;
				}
				case 'K':
				{
					float stepPosition = 0.7;
					float stepRotation = 0.4;
					float errorPos = 1, errorQ = 1;
					Eigen::Matrix4f pose = arm.pose();;
					Eigen::Vector3f incTrans = Eigen::MatrixXf::Random(3,1)*0.05;
					pose.block<3,1>(0,3) += incTrans;
					Eigen::Matrix3f m;
					m = Eigen::AngleAxisf	((double(rand())/RAND_MAX - 0.5)*1, 	Eigen::Vector3f::UnitX())
						* Eigen::AngleAxisf	((double(rand())/RAND_MAX - 0.5)*1,  	Eigen::Vector3f::UnitY())
						* Eigen::AngleAxisf	((double(rand())/RAND_MAX - 0.5)*1, 	Eigen::Vector3f::UnitZ());
					pose.block<3,3>(0,0) = m*pose.block<3,3>(0,0);

					Eigen::Vector3f positionTarget = pose.block<3,1>(0,3);
					Eigen::Quaternionf qTarget(pose.block<3,3>(0,0)); 
					auto handleX = hecatonquiros::ModelSolverOpenRave::drawLine(positionTarget,positionTarget+pose.block<3,1>(0,0)*0.05, 0.002,1,0,0);
					auto handleY = hecatonquiros::ModelSolverOpenRave::drawLine(positionTarget,positionTarget+pose.block<3,1>(0,1)*0.05, 0.002,0,1,0);
					auto handleZ = hecatonquiros::ModelSolverOpenRave::drawLine(positionTarget,positionTarget+pose.block<3,1>(0,2)*0.05, 0.002,0,0,1);

					std::vector<float> ikJoints;
					if(!arm.checkIk(pose, ikJoints, hecatonquiros::ModelSolver::IK_TYPE::IK_6D)){
						std::cout << "Point is not reachable" << std::endl;
						break;
					}

					while(errorPos > 0.01){
						std::vector<float> joints;
						arm.jacobianStep(pose, joints, stepPosition, stepRotation);
						arm.joints(joints);

						auto currentPose = arm.pose();
						auto handleX = hecatonquiros::ModelSolverOpenRave::drawLine(currentPose.block<3,1>(0,3),currentPose.block<3,1>(0,3)+currentPose.block<3,1>(0,0)*0.07, 0.001,1,0,0);
						auto handleY = hecatonquiros::ModelSolverOpenRave::drawLine(currentPose.block<3,1>(0,3),currentPose.block<3,1>(0,3)+currentPose.block<3,1>(0,1)*0.07, 0.001,0,1,0);
						auto handleZ = hecatonquiros::ModelSolverOpenRave::drawLine(currentPose.block<3,1>(0,3),currentPose.block<3,1>(0,3)+currentPose.block<3,1>(0,2)*0.07, 0.001,0,0,1);
						errorPos = (currentPose.block<3,1>(0,3) - positionTarget).norm();
						std::this_thread::sleep_for(std::chrono::milliseconds(30));
					}
					getchar();
					break;
				}
				case 'l':
				{
					float dx, dy, dz, distance, step_size;
					std::cout << "Moving along a line. Pick direction and distance."<<std::endl;
					std::cout << "dx: "; std::cin >> dx;
					std::cout << "dy: "; std::cin >> dy;
					std::cout << "dz: "; std::cin >> dz;
					std::cout << "distance: "; std::cin >> distance;
					std::cout << "step_size: "; std::cin >> step_size;
					if(step_size < 0){
						std::cout << "Bad step size" <<std::endl;
						continue;
					}
					Eigen::Vector3f dir = {dx, dy, dz};
					dir /=dir.norm();
					int iters = distance/step_size;
					for(unsigned i = 0; i < iters; i++){
						auto pose = arm.pose();
						pose.block<3,1>(0,3) +=dir*step_size;
						std::vector<float> joints;
						std::this_thread::sleep_for(std::chrono::milliseconds(30));
						if(arm.checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
							arm.joints(joints, true);
						}else{
							std::cout << "Not found IK" << std::endl;
						}
					}
					break;
				}
				case '+':
				{
					auto poseInit = arm.pose();
					std::cout << "how much time in seconds: ";
					float totalTime = 5000;
					std::cin >> totalTime;
					totalTime *= 1000;
					auto t0 = std::chrono::high_resolution_clock::now();
					for(;;){
						auto t1 = std::chrono::high_resolution_clock::now();
						float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
						if(incT > totalTime)
							break;

						auto pose = poseInit;

						pose(0,3) += sin(incT*0.0/1000)*0.15;
						pose(1,3) += cos(incT*0.5/1000)*0.15;
						pose(2,3) += sin(incT*0.5/1000)*0.15;
						std::vector<float> joints;
						std::this_thread::sleep_for(std::chrono::milliseconds(30));
						if(arm.checkIk(pose, joints, hecatonquiros::ModelSolver::IK_TYPE::IK_3D)){
							arm.joints(joints, true);
						}else{
							std::cout << "Not found IK" << std::endl;
						}
					}
					break;
				}
			}	
		}	
	}
}
