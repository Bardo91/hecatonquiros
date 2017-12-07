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
 
    float y = sqrt(_position[0]*_position[0] + _position[1]*_position[1]);
    float x = _position[2] - mBaseHeight;
 
    float D = (x*x+y*y-mHumerus*mHumerus-mRadius*mRadius)/(2*mHumerus*mRadius);
    float auxD = 1 - D*D;
    if(auxD < 0){
      auxD = 0;
      std::cout << "1-D^2 < 0, unrecheable point, fitting to 0\n";
    }
 
    float theta2a = atan2(sqrt(auxD), D);
    float theta2b = atan2(-sqrt(auxD), D);
 
    float k1a = mHumerus + mRadius*cos(theta2a);
    float k2a = mRadius*sin(theta2a);
    float theta1a = atan2(y,x)-atan2(k2a,k1a);
 
    float k1b = mHumerus + mRadius*cos(theta2b);
    float k2b = mRadius*sin(theta2b);
    float theta1b = atan2(y,x)-atan2(k2b,k1b);
 
    float angleDistA = abs(theta1a - mArmjoints[1]) + abs(theta2a - mArmjoints[2]);
    float angleDistB = abs(theta1b - mArmjoints[1]) + abs(theta2b - mArmjoints[2]);
 
    //if(angleDistA < angleDistB){
        mArmjoints[1] = theta1a;
        mArmjoints[2] = theta2a;
    //}else{
    //    mArmjoints[1] = theta1b;
    //    mArmjoints[2] = theta2b;
    //}
 
    joints(mArmjoints); // send joints to arduino
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
