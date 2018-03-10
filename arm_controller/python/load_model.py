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
from numpy import random, linalg

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('../config/dual_arm_manipulator_5dof.env.xml') # load a simple scene
robot1 = env.GetRobots()[0] # get the first robot
robot2 = env.GetRobots()[1] # get the first robot

with env: # lock the environment since robot will be used
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    
robot1.SetDOFValues([numpy.pi/2,numpy.pi/2,numpy.pi/2,numpy.pi/2,numpy.pi/2],[0,1,2, 3, 4]) # set joint 0 to value 0.5
robot2.SetDOFValues([0,0,numpy.pi/2,0,0],[0,1,2, 3, 4]) # set joint 0 to value 0.5

raveLogInfo("Robot "+robot1.GetName()+" has "+repr(robot1.GetDOF())+" joints with values:\n"+repr(robot1.GetDOFValues()))
raveLogInfo("Robot "+robot2.GetName()+" has "+repr(robot2.GetDOF())+" joints with values:\n"+repr(robot2.GetDOFValues()))

handles = []
for body in robot1.GetLinks():
    T =body.GetTransform()
    handles.append(env.drawlinestrip(points=numpy.array(( T[0:3,3],
                                                          T[0:3,3] + T[0:3,0]*0.1)),
                                    linewidth=3.0,
                                    colors=numpy.array(((1,0,0)))))

    handles.append(env.drawlinestrip(points=numpy.array(( T[0:3,3],
                                                          T[0:3,3] + T[0:3,1]*0.1)),
                                    linewidth=3.0,
                                    colors=numpy.array(((0,1,0)))))


    handles.append(env.drawlinestrip(points=numpy.array(( T[0:3,3],
                                                          T[0:3,3] + T[0:3,2]*0.1)),
                                    linewidth=3.0,
                                    colors=numpy.array(((0,0,1)))))


for body in robot2.GetLinks():
    T =body.GetTransform()
    handles.append(env.drawlinestrip(points=numpy.array(( T[0:3,3],
                                                          T[0:3,3] + T[0:3,0]*0.1)),
                                    linewidth=3.0,
                                    colors=numpy.array(((1,0,0)))))

    handles.append(env.drawlinestrip(points=numpy.array(( T[0:3,3],
                                                          T[0:3,3] + T[0:3,1]*0.1)),
                                    linewidth=3.0,
                                    colors=numpy.array(((0,1,0)))))


    handles.append(env.drawlinestrip(points=numpy.array(( T[0:3,3],
                                                          T[0:3,3] + T[0:3,2]*0.1)),
                                    linewidth=3.0,
                                    colors=numpy.array(((0,0,1)))))

# manip = robot1.SetActiveManipulator('manipulator') # set the manipulator to leftarm
# ikmodel = databases.inversekinematics.InverseKinematicsModel(robot1, iktype=IkParameterization.Type.TranslationDirection5D)
# if not ikmodel.load():
#     ikmodel.autogenerate()

# print("Generated ikmodel")

# while True:
#     with env:
#         target=ikmodel.manip.GetTransform()[0:3,3]
#         print(target)
#         direction = random.rand(3)*0 +  (0,1,0)
#         direction /= linalg.norm(direction)
#         solutions = ikmodel.manip.FindIKSolutions(IkParameterization(Ray(target,direction),IkParameterization.Type.TranslationDirection5D),IkFilterOptions.CheckEnvCollisions)
#         if solutions is not None and len(solutions) > 0: # if found, then break
#             print("Found IK")
#             h=env.drawlinestrip(array([target,target+0.1*direction]),10)
#             for i in random.permutation(len(solutions))[0:min(80,len(solutions))]:
#                 with env:
#                     robot1.SetDOFValues(solutions[i],ikmodel.manip.GetArmIndices())
#                     env.UpdatePublishedBodies()
#                 time.sleep(0.2)
#             h=None
#         else:
#             print("Could not find IK")

manip = robot1.SetActiveManipulator('manipulator') # set the manipulator to leftarm
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot1,iktype=IkParameterization.Type.Translation3D)
if not ikmodel.load():
    ikmodel.autogenerate()

print("Generated IKMODEL")

with robot1: # lock environment and save robot1 state
    robot1.SetDOFValues([0,0,numpy.pi/2],[0,1,2]) # set the first 4 dof values
    Tee = manip.GetEndEffectorTransform() # get end effector
    print(Tee)
    Tee[0][3] = Tee[0][3] +0.1
    Tee[2][3] = Tee[2][3] -0.1
    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.IgnoreSelfCollisions) # get all solutions

print("FOUND SOLUTIONS")

h = env.plot3(Tee[0:3,3],10) # plot one point
with robot1: # save robot1 state
    raveLogInfo('%d solutions'%len(sols))
    for sol in sols: # go through every solution
        robot1.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot1
        time.sleep(100.0/len(sols))

raveLogInfo('restored dof values: '+repr(robot1.GetDOFValues())) # robot1 state is restored to original


IPython.embed()
