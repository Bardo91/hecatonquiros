"""Loads up an environment, attaches a viewer, loads a scene, and requests information about the robot.
"""
from openravepy import *
import time
import IPython

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('../config/dual_arm_manipulator_5dof.env.xml') # load a simple scene
robot1 = env.GetRobots()[0] # get the first robot
robot2 = env.GetRobots()[1] # get the first robot

with env: # lock the environment since robot will be used
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple

    
robot1.SetDOFValues([0.5,1,1],[0,1,2]) # set joint 0 to value 0.5
robot2.SetDOFValues([-0.5,1,1],[0,1,2]) # set joint 0 to value 0.5

raveLogInfo("Robot "+robot1.GetName()+" has "+repr(robot1.GetDOF())+" joints with values:\n"+repr(robot1.GetDOFValues()))
raveLogInfo("Robot "+robot2.GetName()+" has "+repr(robot2.GetDOF())+" joints with values:\n"+repr(robot2.GetDOFValues()))

IPython.embed()
