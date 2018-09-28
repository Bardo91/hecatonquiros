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


#include <Eigen/Eigen>
#include <serial/serial.h>
#include <thread>
#include <mutex>

namespace hecatonquiros {

	struct Potentiometer {
		/// returns map of potentiometer value to angle in radians
		float valToAngle(float _val) {
			return ((_val - intercept)/slope);
		}

		float slope;
		float intercept;
		float offsetDeg;
	};

	class Positioner{
	public:
		/// Construct positioner hal given the port id and baudrate.
		/// \param _port: string with path to serial port file.
		/// \param _baudrate: baudrate for the serial port.
		Positioner(std::string _port, int _baudrate);

		/// Construct positioner hal given a serial connection.
		/// \param _serialPort: already connected serial port.
		Positioner(serial::Serial *_serialPort);

		/// Deinitialize all internal members
		/// \return: true or false if deinitialization of the internal member went good or bad
		bool close();

		/// Returns the relative position of the hand from the base of the positioner
		/// \param _x: 
		/// \param _y: 
		/// \param _z: 
		void baseToHand (float &_x, float &_y, float &_z);
		
		/// Returns the relative position of the hand from the base of the positioner
		/// \param _position: vector containting the 3D position
		void baseToHand (Eigen::Vector3f &_position);

		/// Returns the relative pose of the hand from the base of the positioner
		/// \param _pose: vector containting the 3D pose
		void baseToHand (Eigen::Matrix4f &_pose);
		
		/// Returns the relative position of the base from the handle of the positioner
		/// \param _x: 
		/// \param _y: 
		/// \param _z:
		void handToBase (float &_x, float &_y, float &_z);
		
		/// Returns the relative position of the base from the handle of the positioner
		/// \param _position: vector containting the 3D position
		void handToBase (Eigen::Vector3f &_position);
		
		/// Returns the relative pose of the base from the handle of the positioner
		/// \param _pose: vector containting the 3D pose
		void handToBase (Eigen::Matrix4f &_pose);
		

		/// Returns the raw values of the potentiomenters from 0 to 1023
		/// \param _j0: 
		/// \param _j1: 
		/// \param _j2: 
		/// \param _j3: 
		/// \param _j4:
		void rawJoints  (float &_j0, float &_j1, float &_j2, float &_j3, float &_j4);

		/// Returns the raw values of the potentiomenters from 0 to 1023
		std::vector<float> rawJoints  ();
		
		/// Returns the angles of the joints
		/// \return angles of the potentiometer in a vector
		std::vector<float> angles     ();

		/// Returns the direction of the acceleration vector measured in the base of the positioner.
		/// \return 3d vector containing the direction of the acceleration vector
		Eigen::Vector3f accelerationVector();

		/// Toggle base lock
		void toggleLock();

		/// Returns the transforms of the joints
		/// \param _T01:
		/// \param _T12:
		/// \param _T23:
		/// \param _T34: 
		void lastTransforms(Eigen::Matrix4f &_T01, Eigen::Matrix4f &_T12, Eigen::Matrix4f &_T23, Eigen::Matrix4f & _T34, Eigen::Matrix4f & _T4f);

	private:
		/// Initialize all the internal members
		void init();

	private:
		serial::Serial	*mArduinoCom;
		std::mutex		mSecureRead;
		std::thread		mSerialThread;
		bool			mRun = true;
		bool 			mToggleLock = false;

		float mJ0, mJ1, mJ2, mJ3, mJ4;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> mT01, mT12, mT23, mT34, mT4f;
		Potentiometer mP0, mP1, mP2, mP3, mP4;
		Eigen::Vector3f mAccelerationDirection;

		const float cL01 = 0.066f; //0.068f;
		const float cL12 = 0.104f; //0.169f;
		const float cL23 = 0.153f; //0.169f;
		const float cL34 = 0.068f; //0.142f;
		const float cL4f = 0.08f; //0.08f;
	};
}