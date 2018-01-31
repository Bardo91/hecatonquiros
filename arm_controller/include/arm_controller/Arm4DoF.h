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

#include <arm_controller/backends/Backend.h>

#include <string>
#include <Eigen/Eigen>
#include <math.h>

namespace hecatonquiros{
    class Arm4DoF {
    public:
        /// Constructor 
        Arm4DoF(const Backend::Config &_config);

        /// Send robot to home.
        void home();

        /// set longitudes
        void barsSize(float _base, float _bar1, float _bar2){
            mBaseHeight = _base;
            mHumerus = _bar1;
            mRadius = _bar2;
        }

        /// Joints in radians
        void joints(std::vector<float>);
        std::vector<float> joints() const;

        /// Position in cartesian coordinates (meters)
        void position(Eigen::Vector3f _position, float _wirst = 0);
        Eigen::Vector3f position();

        Eigen::Matrix4f pose() const;

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Vector3f _position);

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Vector3f _position, std::vector<Eigen::Matrix4f> &_transformations);

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Vector3f _position, std::vector<float> &_angles);

        /// Position in cartesian coordinates (meters)
        bool checkIk(Eigen::Vector3f _position, std::vector<float> &_angles, std::vector<Eigen::Matrix4f> &_transformations);

        /// Compute DK and check if valid
        bool directKinematic(const std::vector<float> &_angles, std::vector<Eigen::Matrix4f> &_transformations);

        //void lastTransformations(Eigen::Matrix4f &_t01, Eigen::Matrix4f &_t12, Eigen::Matrix4f &_t23, Eigen::Matrix4f &_t34);
        void lastTransformations(Eigen::Matrix4f &_t0, Eigen::Matrix4f &_t1, Eigen::Matrix4f &_t2);

        void closeClaw();
        void openClaw();
        void stopClaw();

        bool isInit() const;
    private:
        Backend *mBackend;

        std::vector<float> mArmjoints = std::vector<float>(4);

        Eigen::Matrix<float,4,4,Eigen::DontAlign> mT01, mT12, mT23, mT34;
        Eigen::Matrix<float,4,4,Eigen::DontAlign> mFinalT;

        float   mHome1 = 0*M_PI/180.0,
                mHome2 = 0*M_PI/180.0,
                mHome3 = 90*M_PI/180.0,
                mHome4 = 90*M_PI/180.0,
                mHome5 = 90*M_PI/180.0;

        double mHumerus = 0.15;
        double mRadius = 0.09;
        double mBaseHeight = 0.08;

        int mArmId = 1;
    };
}