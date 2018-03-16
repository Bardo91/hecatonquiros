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

#include <hecatonquiros/model_solvers/ModelSolverSimple4Dof.h>

namespace hecatonquiros{
    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverSimple4Dof::joints(const std::vector<float> &_joints){
        mJoints = _joints;
        mUpdatedTransforms = false;
    }

    //-----------------------------------------------------------------------------------------------------------------
    std::vector<float> ModelSolverSimple4Dof::joints() const{
        return mJoints;
    }

    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverSimple4Dof::jointsTransform(std::vector<Eigen::Matrix4f> &_transforms) {
        if(!mUpdatedTransforms)
            updateTransforms();

        _transforms.clear();
        for(auto &t: mJointsTransform){
            _transforms.push_back(t);
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    Eigen::Matrix4f ModelSolverSimple4Dof::jointTransform(int _idx) {
        if(!mUpdatedTransforms)
            updateTransforms();

        if(_idx < mJointsTransform.size()){
            return mJointsTransform[_idx];
        }else{
            return Eigen::Matrix4f::Identity();
        }
    }

    //-----------------------------------------------------------------------------------------------------------------
    bool ModelSolverSimple4Dof::checkIk(const Eigen::Matrix4f &_pose, std::vector<float> &_joints, bool _forceOri){
        _joints.resize(mJoints.size());

        Eigen::Vector3f position = _pose.block<3,1>(0,3);

        _joints[0] = atan2(position[1], position[0]);
    
        float y = sqrt(position[0]*position[0] + position[1]*position[1]);
        float x = position[2] - mBaseHeight;
    
        float D = (x*x+y*y-mHumerus*mHumerus-mRadius*mRadius)/(2*mHumerus*mRadius);
        float auxD = 1 - D*D;
        if(auxD < 0){ // NON REACHABLE POINT
            //std::cout << "1-D^2 < 0, unrecheable point, fitting to 0\n";
            return false;
        }else{  // REACHABLE POINT
            float theta0a = atan2(position[1], position[0]);
            float theta2a = atan2(sqrt(auxD), D);

            float k1a = mHumerus + mRadius*cos(theta2a);
            float k2a = mRadius*sin(theta2a);
            float theta1a = atan2(y,x)-atan2(k2a,k1a);
            _joints = {theta0a, theta1a, theta2a, 90.0*M_PI/180.0};

            return true;
        } 
    }

    //-----------------------------------------------------------------------------------------------------------------
    void ModelSolverSimple4Dof::updateTransforms(){
        if(mJointsTransform.size() < mJoints.size()){
            mJointsTransform.resize(mJoints.size());
        }
        Eigen::Matrix4f t01, t12, t23, t34;

        t01 <<	cos(mJoints[0]),  -sin(mJoints[0]),  0, 0,
                sin(mJoints[0]),  cos(mJoints[0]), 0, 0,
                0,          0,  1,         mBaseHeight,
                0,          0,  0,         1;

        t12 <<	cos(-M_PI/2 + mJoints[1]), 0, sin(-M_PI/2 + mJoints[1]), mHumerus * cos(-M_PI/2 + mJoints[1]),
                0,1,0,0,
                -sin(-M_PI/2 + mJoints[1]), 0, cos(-M_PI/2 + mJoints[1]),  -mHumerus * sin(-M_PI/2 + mJoints[1]),
                0,                  0,                  0,  1;

        t23 <<	cos(mJoints[2]),  0, sin(mJoints[2]),   mRadius * cos(mJoints[2]),
                0,1,0,0,
                -sin(mJoints[2]), 0,  cos(mJoints[2]),    -mRadius * sin(mJoints[2]),
                0,         0,          0,   1;


        t34 = Eigen::Matrix4f::Identity();

        mJointsTransform[0] = t01;
        mJointsTransform[1] = t01*t12;
        mJointsTransform[2] = t01*t12*t23;
        mJointsTransform[4] = t01*t12*t23*t34;
        mFinalT = t01*t12*t23*t34;

        /*if( (_angles[0]>=(-90*M_PI/180) && _angles[0]<=(90*M_PI/180)) && 
            (_angles[1]>=(-90*M_PI/180) && _angles[1]<=(90*M_PI/180)) && 
            (_angles[2]>=(-90*M_PI/180) && _angles[2]<=(90*M_PI/180)) ){
            // Perfectly reachable point
            return true;
        }
        else{   
            // Exceeding some joint max angle
            return false;
        }*/
    }

}