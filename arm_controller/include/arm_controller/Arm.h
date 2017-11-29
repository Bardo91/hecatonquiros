//
//
//
//
//

#include <string>
#include <serial/serial.h>
#include <Eigen/Eigen>
#include <math.h>

class Arm {
public:
    /// Constructor getting authority of serial port.
    Arm(std::string &_port, int _baudrate, int _id = 1);

    /// Constructor for sharing serial port
    Arm(serial::Serial *_serialPort, int _id = 1);

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
    void position(Eigen::Vector3f _position);
    Eigen::Vector3f position();

	Eigen::Matrix4f pose() const;

    //void lastTransformations(Eigen::Matrix4f &_t01, Eigen::Matrix4f &_t12, Eigen::Matrix4f &_t23, Eigen::Matrix4f &_t34);
    void lastTransformations(Eigen::Matrix4f &_t0, Eigen::Matrix4f &_t1, Eigen::Matrix4f &_t2);

    void closeClaw();
    void openClaw();
    void stopClaw();
    void actuateWrist(float _actionWrist);

private:
    void sendCurrentJoints();

private:
	serial::Serial	*mArduinoCom;
    std::vector<float> mArmjoints = std::vector<float>(3);

    Eigen::Matrix<float,4,4,Eigen::DontAlign> mT01, mT12, mT23;
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
