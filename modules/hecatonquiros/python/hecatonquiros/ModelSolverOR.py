#---------------------------------------------------------------------------------------------------------------------
#  HECATONQUIROS
#---------------------------------------------------------------------------------------------------------------------
#  Copyright 2018 ViGUS University of Seville
#---------------------------------------------------------------------------------------------------------------------
#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
#  and associated documentation files (the "Software"), to deal in the Software without restriction, 
#  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
#  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all copies or substantial 
#  portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
#  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
#  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
#  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
#  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#---------------------------------------------------------------------------------------------------------------------

import numpy as np
from enum import Enum

from openravepy import *
from ModelSolver import ModelSolver

import IPython

class IK_TYPE(Enum):
    IK_3D = IkParameterization.Type.Translation3D
    IK_4D = IkParameterization.Type.TranslationXAxisAngle4D
    IK_5D = IkParameterization.Type.TranslationDirection5D
    IK_6D = IkParameterization.Type.Transform6D
    IK_LOOK = IkParameterization.Type.Lookat3D
    
class ModelSolverOR(ModelSolver):

    def __init__(self, _data):
        self.data_=_data
        self.orEnv_ = Environment() # create openrave environment
        RaveSetDebugLevel(0)
        if(_data["visualize"]):
            self.orEnv_.SetViewer('qtcoin') # attach viewer (optional)
        
        self.orEnv_.Load("/home/bardo91/programming/hecatonquiros/modules/hecatonquiros/config/empty.env.xml")
        self.orEnv_.GetViewer().SetCamera(np.array([    [ 0.76120956 ,0.10644552 , 0.63971037, -0.48476961],
                                                        [ 0.62537234,  0.14063313, -0.76754919 , 0.60650802],
                                                        [-0.17166664,  0.98432295 , 0.0404832  , 0.14137109],
                                                        [ 0.         , 0. ,         0. ,         1.        ]] ))

        robot = self.orEnv_.ReadRobotXMLFile(_data["robot_file"]) # load a simple scene
        if(not robot):
            raise BaseException("Bad arm file")
        self.orEnv_.Add(robot)
        self.robot_ = self.orEnv_.GetRobots()[0] # get the first robot

        if(_data["enable_physics"]):
            with self.orEnv_:
                self.physics_ = RaveCreatePhysicsEngine(self.orEnv_,'ode')
                self.orEnv_.SetPhysicsEngine(self.physics_)
                self.orEnv_.GetPhysicsEngine().SetGravity([0,0,-9.8])
                # Work with ideal controller to prevent it to fall due to gravity
                self.armController_ = RaveCreateController(self.orEnv_,"idealcontroller")
                self.robot_.SetController(self.armController_, range(0,self.robot_.GetDOF()), 1)
                # Restart simulation
                self.orEnv_.StopSimulation()
                self.orEnv_.StartSimulation(timestep=0.001)

        with self.orEnv_: # lock the environment since robot will be used
            self.viewer_ = self.orEnv_.GetViewer()
            self.viewer_.SetBkgndColor([.8, .85, .9])  # RGB tuple


        self.robotManip_ = self.robot_.SetActiveManipulator('manipulator') # Generalize!

        # initialize dict with IK_TYPES.
        self.ikModels = {}
        for ik in IK_TYPE:
            self.ikModels[ik] = None

        self.handles_ = []

    def __checkIkDbs(self, _ikType):
        if(self.ikModels[_ikType] == None):
            self.ikModels[_ikType] = databases.inversekinematics.InverseKinematicsModel(self.robot_, iktype=_ikType.value)
            if not self.ikModels[_ikType].load():
                print("Failed to find IK solver for requested type. Creating solver, it might take some time")
                self.ikModels[_ikType].autogenerate()


    ### Set joints of robot
    ### \param _joints: desired joints
    def setJoints(self, _joints):
        if(self.data_["enable_physics"]):
            self.armController_.SetDesired(_joints)
        else:
            self.robot_.SetJointValues(_joints)

        self.orEnv_.UpdatePublishedBodies()
    
    ### Get current joints of robot
    def getJoints(self):
        return [x.GetValue(0) for x in self.robot_.GetJoints()] 

    ### Get transforms of joints
    def jointsTransform(self):
        T = []
        for body in self.robot_.GetLinks():
            T.append(body.GetTransform())
        return T

    def endEffectorPosition(self):
        return self.jointsTransform()[-1][0:3,3]

    ### Get transforms of specific joint
    def jointTransform(self, _idx):
        return self.robot_.GetLinks()[_idx].GetTransform()

    ### Check if exists IK for a given pose
    ### \param _pose: desired pose. If 5DoF, the Z axis is used as target direction.
    ### \param _joints: joints for given pose
    ### \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
    def checkIk(self, _target, _type = IK_TYPE.IK_3D):
        self.__checkIkDbs(_type)
        solution = self.robotManip_.FindIKSolution( IkParameterization(_target, _type.value),
                                                    IkFilterOptions.CheckEnvCollisions)

        return solution!=None, solution

    ### Check if exists IK for a given pose
    ### \param _pose: desired pose. If 5DoF, the Z axis is used as target direction.
    ### \param _joints: list of possible solutions joints for given pose
    ### \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
    def checkIks(self, _target, _type = IK_TYPE.IK_3D):
        solutions = self.robotManip_.FindIKSolutions(   IkParameterization(_target, _type.value),
                                                        IkFilterOptions.CheckEnvCollisions)
        return solutions!=None, solutions

    ### End effector pose given joints
    ### \param _joints: joints to test endeffector
    def testFK(self, _joints):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return np.identity(3) 

    ### Get the points of the desired trajectory
    ### \param _pose: desired points that the trajectory must have
    ### \param _traj: list of possible solutions joints for given poses
    ### \param _time: total time of the trajectory
    def getPointsTrajectory(self, _pose):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return False, [[]], 0.0

    ### Get cartesian jacobian
    def jacobian(self):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return np.array([])

    ### Get rotation jacobian in quaternions (X, Y, Z, W)
    def rotationJacobian(self):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return np.array([])
    
    ### Get rotation jacobian in angle axis (unknown angle order!! WARNING)
    def angularRotationJacobian():
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return np.array([])

    def drawPoint(self, _point, _color = (1,0,0), _pointSize = 15):
        self.handles_.append(self.orEnv_.plot3( points=np.array(_point),
                                                pointsize=_pointSize,
                                                colors=np.array(_color)))
        return len(self.handles_)-1

    def removeElement(self, _idx):
        del self.handles_[_idx]