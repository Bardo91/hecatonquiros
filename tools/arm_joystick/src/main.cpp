//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Manuel Perez Jimenez (a.k.a. manuoso) manuperezj@gmail.com
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

#include "ArmJoystick.h"

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char **_argv) {

	ArmJoystick controller;

    ros::init(_argc, _argv, "arm_joystick");
    
    ros::AsyncSpinner rosSpinner(4); // Use 4 thread
    rosSpinner.start();

    if(!controller.init(_argc, _argv)){
        std::cout << "Error initializing Arm Joystick" << std::endl;
        return false;
    }

    controller.start();

    while(ros::ok()){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    controller.stop();

	std::cout << "Finish Main Thread" << std::endl;

    return 0;
}


