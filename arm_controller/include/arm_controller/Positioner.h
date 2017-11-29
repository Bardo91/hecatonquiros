//
// Positioner end-tool code
//
//
//
//

#include <Eigen/Eigen>
#include <serial/serial.h>
#include <thread>
#include <mutex>

struct Potentiometer {
	float valToAngle(float _val) {
		return _val / 1023 * (mMaxAngle - mMinAngle) + mMinAngle;
	}

	float mMinAngle;
	float mMaxAngle;
};

class Positioner{
public:
	Positioner(std::string _port, int _baudrate);
	Positioner(serial::Serial *_serialPort);

    void init();

	bool close();

	void baseToHand (float &_x, float &_y, float &_z);
	void handToBase (float &_x, float &_y, float &_z);
	void rawJoints  (float &_j0, float &_j1, float &_j2, float &_j3, float &_j4);
	void angles     (float &_t0, float &_t1, float &_t2, float &_t3, float &_j4);
    std::vector<float> angles     ();

	void lastTransforms(Eigen::Matrix4f &_T01, Eigen::Matrix4f &_T12, Eigen::Matrix4f &_T23, Eigen::Matrix4f & _T34, Eigen::Matrix4f & _T4f);

private:
	serial::Serial	*mArduinoCom;
	std::mutex		mSecureRead;
	std::thread		mSerialThread;
	bool			mRun = true;

	float mJ0, mJ1, mJ2, mJ3, mJ4;
	Eigen::Matrix4f mT01, mT12, mT23, mT34, mT4f;
	Potentiometer mP0, mP1, mP2, mP3, mP4;

	const float cL01 = 0.066f; //0.068f;
	const float cL12 = 0.104f; //0.169f;
	const float cL23 = 0.153f; //0.169f;
    const float cL34 = 0.068f; //0.142f;
    const float cL4f = 0.08f; //0.08f;
};

