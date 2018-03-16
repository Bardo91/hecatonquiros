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

#include <hecatonquiros/model_solvers/ModelSolverOpenRave.h>
#include <hecatonquiros/model_solvers/ModelSolverSimple4Dof.h>
#include <hecatonquiros/model_solvers/ModelSolverRos.h>

namespace hecatonquiros{
    ModelSolver* ModelSolver::create(const ModelSolver::Config &_config){
        ModelSolver *ms = nullptr;
        switch(_config.type){
        case Config::eType::Simple4DoF:
            ms = new ModelSolverSimple4Dof();
            break;
        case Config::eType::OpenRave:
            ms = new ModelSolverOpenRave();
            break;
        case Config::eType::Ros:
            ms = new ModelSolverRos();
            break;
        }

        if(ms->init(_config)){
            return ms;
        }else{
            return nullptr;
        }
    }

}
