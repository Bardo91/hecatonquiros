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

class ModelSolver:
    class IK_TYPE(Enum):
        IK_3D=3
        IK_4D=4
        IK_5D=5
        IK_6D=6
        IK_LOOK = 31

    ### Set joints of robot
    ### \param _joints: desired joints
    def joints(self, _joints):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
    
    ### Get current joints of robot
    def joints(self):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return []

    ### Get transforms of joints
    def jointsTransform(self):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return []

    ### Get transforms of specific joint
    def jointTransform(self, _idx):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return np.array([])

    ### Check if exists IK for a given pose
    ### \param _pose: desired pose. If 5DoF, the Z axis is used as target direction.
    ### \param _joints: joints for given pose
    ### \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
    def checkIk(self, _pose, _type = IK_TYPE.IK_3D):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return False, []

    ### Check if exists IK for a given pose
    ### \param _pose: desired pose. If 5DoF, the Z axis is used as target direction.
    ### \param _joints: list of possible solutions joints for given pose
    ### \param _forceOri: if true target pose need to be reachable in position and orientation. If false target orientation can be ignored.
    def checkIk(self, _pose, _type = IK_TYPE.IK_3D):
        raise NotImplementedError( "This ModelSolver does not implements set pose method" )
        return False, [[]]

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
