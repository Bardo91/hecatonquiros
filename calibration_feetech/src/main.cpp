//
//
//
//
//



#include <hecatonquiros/backends/dep/SCServo.h>  

#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <mutex>
#include <iomanip>

//--------------------------------------------------------------------------------------------------------------------

std::mutex pointsLock;
int pointToSave = 0;

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "calibration_feetech");
		
	std::cout << "Write serial port (default: /dev/ttyUSB0), type 1 for default: ";
	std::string portUSB;
	std::cin >> portUSB;
	if(portUSB == "1"){
		portUSB = "/dev/ttyUSB0";
	}

	std::cout << "Write ID of Servo Feetech you want to calibrate: ";
	int idServo;
	std::cin >> idServo;

	SCServo *servoDriver;
	std::thread positionCheck;
	bool fin = false;
	bool showPos = false;

	servoDriver =  new SCServo(portUSB);

	if(servoDriver->isConnected()){
            positionCheck = std::thread([&](){
				servoDriver->EnableTorque(idServo, 0);
                while(servoDriver->isConnected() && !fin){
                    
					int pos = servoDriver->ReadPos(idServo);

					if(showPos){
						std::cout << "Servo " << idServo << " Position:  " << pos << std::endl;
						showPos = false;
					}

					pointsLock.lock();
					pointToSave = pos;
					pointsLock.unlock();

                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            });
    }else{
        std::cout << "Servo Driver not connected!" << std::endl;
    }

	std::vector<int> savedPoints, xPoints;
	std::vector<int> yPoints{-90, 0, 90};
	float m, b, n, corteEjeY2, sumXPoints = 0.0, sumYPoints = 0.0, sumXYPoints = 0.0, sumXXPoints = 0.0;

	char c;
	while(!fin) {
		std::cin >> c;
		switch(c){
			case 'p':
				std::cout << "Show Position Servo" <<std::endl;
				showPos = true;
				break;	
			case 's':
				std::cout << "Saved point" <<std::endl;
				pointsLock.lock();
				savedPoints.push_back(pointToSave);
				pointsLock.unlock();
				if(savedPoints.size() > 3){
					std::cout << "More points saved than allowed, please delate all and start again" << std::endl;
				}
				break;
			case 'c':
				std::cout << "Compute saved points" << std::endl;
				xPoints = savedPoints;		// You can copy a vector with = , std::copy, iterative or passing it as constructor
				for(int i = 0; i < yPoints.size(); i++){
					sumXPoints = sumXPoints + xPoints[i];
					sumYPoints = sumYPoints + yPoints[i];
					sumXYPoints = sumXYPoints + xPoints[i]*yPoints[i];
					sumXXPoints = sumXXPoints + xPoints[i]*xPoints[i];
				}
				n = yPoints.size();
				/* Debug */
				//std::cout << sumXPoints << " | " << sumYPoints << " | " << sumXYPoints << " | " << sumXXPoints << " | " << yPoints.size() << std::endl;
				/* End Debug */
				// y = m*x + b
				m = (sumXYPoints - ((sumXPoints*sumYPoints)/n))/(sumXXPoints - ((sumXPoints*sumXPoints)/n));
				b = (sumYPoints/n) - ((m*sumXPoints)/n);
				corteEjeY2 = m*1023.0 + b;
				std::cout << std::fixed << std::setprecision(3) << "Recta obtenida, y = m*x + b " << std::endl;
				std::cout << std::fixed << std::setprecision(3) << "y = " << m << " *x + " << b << std::endl;
				std::cout << std::fixed << std::setprecision(3) << "Corte Eje Y, X = 0: " << b << std::endl;
				std::cout << std::fixed << std::setprecision(3) << "Corte Eje Y, X = 1023: " << corteEjeY2 << std::endl;
				break;
			case 'd':
				std::cout << "Delete all saved points" << std::endl;
				for(int i = 0 ; i < savedPoints.size() ; i++){
                	savedPoints[i] = 0;
            	}
				break;
			case 'v':
				std::cout << "View all saved points" << std::endl;
				std::cout << " | ";
				for(int i = 0 ; i < savedPoints.size() ; i++){
                	std::cout << savedPoints[i] << " | ";
            	}
				std::cout << std::endl;
				break;
			case 'e':
				std::cout << "Pulsed exit program" << std::endl;
				fin = true;
				break;
			
		}	
	}	
	return 0;
}
