//
//
//
//
//
//

#include <arm_controller/Arm.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

//---------------------------------------------------------------------------------------------------------------------
Arm::Arm(std::string & _port, int _baudrate, int _id) {
    mArduinoCom = new serial::Serial(_port, _baudrate, serial::Timeout::simpleTimeout(1000));
    mArmId = _id;
    home();
}

//---------------------------------------------------------------------------------------------------------------------
Arm::Arm(serial::Serial * _serialPort, int _id) {
    mArduinoCom = _serialPort;
    mArmId = _id;
    home();
}

//---------------------------------------------------------------------------------------------------------------------
void Arm::home(){
  joints({mHome1, mHome2, mHome3});
}

//---------------------------------------------------------------------------------------------------------------------
void Arm::joints(std::vector<float> _q) {
    mArmjoints[0] = _q[0];
    mArmjoints[1] = _q[1];
    mArmjoints[2] = _q[2];
    sendCurrentJoints(); // SEND joints to arduino
}

//---------------------------------------------------------------------------------------------------------------------
std::vector<float> Arm::joints() const {
    return mArmjoints;
}


#include <opencv2/opencv.hpp>
//---------------------------------------------------------------------------------------------------------------------
void Arm::position(Eigen::Vector3f _position) {
    mArmjoints[0] = atan2(_position[1], _position[0]);

    float x = sqrt(_position[0]*_position[0] + _position[1]*_position[1]);
    if(_position[0] < 0){
        x *= -1;
    }
    float z = _position[2] - mBaseHeight;

    float maxDistance = mHumerus + mRadius -0.01 ;
    float minDistance = mHumerus - mRadius + 0.01;
    float targetDistance = sqrt(x*x+z*z);
    if(targetDistance > maxDistance){
        Eigen::Vector2f dir = {x, z};
        dir /= dir.norm();
        dir *= maxDistance;
        x = dir[0];
        z = dir[1];
    }else if(targetDistance < minDistance){
        Eigen::Vector2f dir = {x, z};
        dir /= dir.norm();
        dir *= minDistance;
        x = dir[0];
        z = dir[1];
    }

    float errorAbs = 9999;
    while(errorAbs > 0.005){
        cv::Mat display = cv::Mat::zeros(500,500,CV_8UC3);
        double max = 0.5, min = -0.5;

        Eigen::Matrix2f J;
        J(0,0) = mHumerus*cos(mArmjoints[1]) + mRadius*cos(mArmjoints[1]+mArmjoints[2]);
        J(1,0) = -mHumerus*sin(mArmjoints[1]) - mRadius*sin(mArmjoints[1]+mArmjoints[2]);
        J(0,1) = mRadius*cos(mArmjoints[1]+mArmjoints[2]);
        J(1,1) = -mRadius*sin(mArmjoints[1]+mArmjoints[2]);

        Eigen::Matrix2f Jinv = J.inverse();

        float xc = mHumerus*sin(mArmjoints[1]) + mRadius*sin(mArmjoints[1]+mArmjoints[2]);
        float zc = mHumerus*cos(mArmjoints[1]) + mRadius*cos(mArmjoints[1]+mArmjoints[2]);


        //float x1 = mHumerus*sin(mArmjoints[1]);
        //float z1 = mHumerus*cos(mArmjoints[1]);
        //cv::circle(display, cv::Point((x-min)/max*250,(z-min)/max*250), 3, cv::Scalar(0,0,255),3);
        //auto p1 = cv::Point((x1-min)/max*250,(z1-min)/max*250);
        //auto p2 = cv::Point((xc-min)/max*250,(zc-min)/max*250);
        //cv::line(display, cv::Point(250,250), p1, cv::Scalar(255,0,0),2);
        //cv::line(display, p1, p2, cv::Scalar(0,255,0),2);
        //cv::imshow("display", display);
        //cv::waitKey(30);
        

        //std::cout << x << "/" << xc << ", " <<z<<"/"<< zc << std::endl;

        Eigen::Vector2f error = {
            x-xc,
            z-zc
        };
        errorAbs = error.norm();
        Eigen::Vector2f incAngles = 0.05*(Jinv*error);

        mArmjoints[1] += incAngles[0];
        mArmjoints[2] += incAngles[1];

        //std::cout << "JOINTS: "<< mArmjoints[1]*180/M_PI << ", " <<mArmjoints[2]*180/M_PI  << std::endl;

        joints(mArmjoints); // send joints to arduino
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

//---------------------------------------------------------------------------------------------------------------------
Eigen::Vector3f Arm::position() {
    float mA0 = mArmjoints[0];
    float mA1 = mArmjoints[1];
    float mA2 = mArmjoints[2];

    mT01 <<	cos(mA0),   0,  sin(mA0),  0,
            sin(mA0),   0,  -cos(mA0), 0,
            0,          1,  0,         mBaseHeight,
            0,          0,  0,         1;

    mT12 <<	cos(M_PI/2 + mA1),  -sin(M_PI/2 + mA1),  0,  mHumerus * cos(M_PI/2 + mA1),
            sin(M_PI/2 + mA1),  cos(M_PI/2 + mA1),  0,  mHumerus * sin(M_PI/2 + mA1),
            0,                  0,                  1,  0,
            0,                  0,                  0,  1;

    mT23 <<	cos(mA2),  -sin(mA2),  0,   mRadius * cos(mA2),
            sin(mA2),  cos(mA2),   0,   mRadius * sin(mA2),
            0,         0,          1,   0,
            0,         0,          0,   1;

    mFinalT = mT01*mT12*mT23;

    return mFinalT.block<3,1>(0,3);
}

//---------------------------------------------------------------------------------------------------------------------
Eigen::Matrix4f Arm::pose() const {
    return Eigen::Matrix4f();
}
//---------------------------------------------------------------------------------------------------------------------
void Arm::lastTransformations(Eigen::Matrix4f &_t0, Eigen::Matrix4f &_t1, Eigen::Matrix4f &_t2) {
    position();
    _t0 = mT01;
    _t1 = mT12;
    _t2 = mT23;
}

//---------------------------------------------------------------------------------------------------------------------
void Arm::sendCurrentJoints() {
    std::string cmd;
    std::stringstream order;
    order << "a"<< mArmId << mArmjoints[0]*180.0/M_PI << "," << mArmjoints[1]*180.0/M_PI << "," << mArmjoints[2]*180.0/M_PI << "\r\n";
    cmd = order.str();
    mArduinoCom->write(cmd);
}

//---------------------------------------------------------------------------------------------------------------------
void Arm::closeClaw(){
    std::string cmd;
    std::stringstream order;
    order << "c"<< mArmId << "c\r\n";
    cmd = order.str();
    mArduinoCom->write(cmd);
}

//---------------------------------------------------------------------------------------------------------------------
void Arm::openClaw(){
    std::string cmd;
    std::stringstream order;
    order << "c"<< mArmId << "o\r\n";
    cmd = order.str();
    mArduinoCom->write(cmd);
}

//---------------------------------------------------------------------------------------------------------------------
void Arm::stopClaw(){
    std::string cmd;
    std::stringstream order;
    order << "c"<< mArmId << "s\r\n";
    cmd = order.str();
    mArduinoCom->write(cmd);
}

//---------------------------------------------------------------------------------------------------------------------
void Arm::actuateWrist(float _actionWrist) {
    std::string cmd;
    std::stringstream order;
    order << "w"<< mArmId << _actionWrist << "\r\n";
    cmd = order.str();
    mArduinoCom->write(cmd);
}
