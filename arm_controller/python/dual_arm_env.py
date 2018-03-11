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

from openravepy import *
import time
import IPython
import numpy as np
import arm_server
import rospy
import threading

rospy.init_node('dual_arm_openrave_environment',anonymous=True)

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/bardo91/programming/catkin_positioner/src/PositionerEndTool/hecatonquiros/arm_controller/config/dual_arm_manipulator_5dof.env.xml') # load a simple scene

with env: # lock the environment since robot will be used
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple

leftArm = arm_server.ArmServer (    env, 
                                    #"/home/bardo91/programming/catkin_positioner/src/PositionerEndTool/hecatonquiros/arm_controller/config/arm_gripper_5dof.robot.xml", 
                                    "",
                                    "left_arm", 
                                    np.array([0.20,0.14,-0.04]),
                                    np.array([0,0,1,0]))   # {w,x,y,z} 180 over Y

rightArm = arm_server.ArmServer (   env, 
                                    #"/home/bardo91/programming/catkin_positioner/src/PositionerEndTool/hecatonquiros/arm_controller/config/arm_gripper_5dof.robot.xml", 
                                    "",
                                    "right_arm", 
                                    np.array([0.2,-0.2,-0.04]),
                                    np.array([0,0,1,0]))   # {w,x,y,z} 180 over Y


spinThread = threading.Thread(target=lambda: rospy.spin(), name="spin_thread")

IPython.embed()

leftArm.stop()
rightArm.stop()