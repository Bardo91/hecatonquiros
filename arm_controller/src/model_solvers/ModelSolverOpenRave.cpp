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

#include <arm_controller/model_solvers/ModelSolverOpenRave.h>

using namespace OpenRAVE;

namespace hecatonquiros{

    bool ModelSolverOpenRave::init(){
        #ifdef HAS_OPENRAVE
            std::string robotname = "";
            std::string iktype = "";
            RaveInitialize(true); // start openrave core

            EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
            RobotBasePtr probot = penv->ReadRobotXMLFile(robotname);
            if( !probot ) {
                penv->Destroy();
                return 2;
            }
            penv->Add(probot);
            ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
            penv->Add(pikfast,true,"");
            std::stringstream ssin,ssout;
            ssin << "LoadIKFastSolver " << probot->GetName() << " " << iktype;
            // if necessary, add free inc for degrees of freedom
            //ssin << " " << 0.04f;
            // get the active manipulator
            RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
            if( !pikfast->SendCommand(ssout,ssin) ) {
                RAVELOG_ERROR("failed to load iksolver\n");
                penv->Destroy();
                return 1;
            }
            RAVELOG_INFO("testing random ik\n");
            while(1) {
                Transform trans;
                trans.rot = quatFromAxisAngle(Vector(RaveRandomFloat()-0.5,RaveRandomFloat()-0.5,RaveRandomFloat()-0.5));
                trans.trans = Vector(RaveRandomFloat()-0.5,RaveRandomFloat()-0.5,RaveRandomFloat()-0.5)*2;
                std::vector<dReal> vsolution;
                if( pmanip->FindIKSolution(IkParameterization(trans),vsolution,IKFO_CheckEnvCollisions) ) {
                    std::stringstream ss; ss << "solution is: ";
                    for(size_t i = 0; i < vsolution.size(); ++i) {
                        ss << vsolution[i] << " ";
                    }
                    ss << std::endl;
                    RAVELOG_INFO(ss.str());
                }
                else {
                    // could fail due to collisions, etc
                }
            }
        #endif 
    }
}