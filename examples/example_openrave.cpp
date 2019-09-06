#include <hecatonquiros/model_solvers/ModelSolverOpenRave.h>


int main(int _argc, char *_argv){

    if(_argc != 2 ){
        std::cerr << "Bad input arguments, run it with just the path to a robot file" << std::endl;
    }

    hecatonquiros::ModelSolver::Config config;
    config.type = hecatonquiros::ModelSolver::Config::eType::OpenRave;
    config.robotName = "arm";
    config.manipulatorName = "manipulator";
    config.robotFile = _argv[1];
    config.offset = {0,0,0};    // x,y,z
    config.rotation = {1,0,0,0};    // w, x, y, z
    config.visualizer = true;


    for(;;){
        
    }

}