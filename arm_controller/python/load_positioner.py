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
env.Load('../config/positioner.env.xml') # load a simple scene
robot1 = env.GetRobots()[0] # get the first robot

with env: # lock the environment since robot will be used
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    
raveLogInfo("Robot "+robot1.GetName()+" has "+repr(robot1.GetDOF())+" joints with values:\n"+repr(robot1.GetDOFValues()))

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
                                    
IPython.embed()
