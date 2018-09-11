//
//
//
//
//
//

#include <QApplication>
#include "PidTuneGui.h"
#include <ros/ros.h>
#include <thread>

int main(int _argc, char **_argv){
    ros::init(_argc, _argv, "PidTuneGui");
    std::thread spinThread([&](){
        ros::spin();
    });

    QApplication a(_argc, _argv);
    PidTuneGui gui( "/aeroarms/crawler_detection/fcu_crawler_pose", 
                    "/aeroarms/mav_controller/ref_fcu_crawler_pose",
                    {   std::pair<std::string, std::string>("X", "/aeroarms/mav_controller/pid_x"),
                        std::pair<std::string, std::string>("Y", "/aeroarms/mav_controller/pid_y"),
                        std::pair<std::string, std::string>("Z", "/aeroarms/mav_controller/pid_z")});
    gui.show();
    
    return a.exec();
}
