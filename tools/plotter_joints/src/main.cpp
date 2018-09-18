//
//
//
//
//
//

#include <QApplication>
#include "PlotterJointsGui.h"
#include <ros/ros.h>
#include <thread>

int main(int _argc, char **_argv){
    ros::init(_argc, _argv, "plotter_joints");
    std::thread spinThread([&](){
        ros::spin();
    });

    QApplication a(_argc, _argv);
    PlotterJointsGui gui( "/hecatonquiros/left_arm/in/target_joints", 
                    "/hecatonquiros/left_arm/out/joints_state");
    gui.show();
    
    return a.exec();
}
