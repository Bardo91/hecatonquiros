//
//
//
//
//
//

#include <QApplication>
#include "PlotterPosesGui.h"
#include <ros/ros.h>
#include <thread>

int main(int _argc, char **_argv){
    ros::init(_argc, _argv, "plotter_poses");
    std::thread spinThread([&](){
        ros::spin();
    });

    QApplication a(_argc, _argv);
    PlotterPosesGui gui( "/hecatonquiros/left_arm/in/target_pose_line3d", 
                    "/hecatonquiros/left_arm/out/pose");
    gui.show();
    
    return a.exec();
}
