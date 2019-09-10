
import IPython

from ModelSolverOR import ModelSolverOR

from openravepy import *
import numpy as np
import time
import math

data = {}
data["visualize"] = True
data["robot_file"]= "/home/bardo-reborn/programming/hecatonquiros/modules/hecatonquiros/config/arm_4dof.robot.xml"
data["enable_physics"] = True
ms = ModelSolverOR(data)

IPython.embed()

counter = 0
rot = 0
rotSpeed = 0.1
start = time.time()
while(True):
    end = time.time()
    if(end-start > 1):
        body = ms.orEnv_.ReadKinBodyXMLFile("/usr/local/share/openrave-0.9/data/lego2.kinbody.xml")
        body.SetName("body{}".format(counter))

        ms.orEnv_.AddKinBody(body,True)
        T = np.eye(4)
        T[0:3,3] = np.array((-0.15,0,1))
        body.SetTransform(T)
        counter = counter +1
        start = end
    rot = rot+rotSpeed
    if rot > math.pi/2 or rot < -math.pi/2:
        rotSpeed*=-1

    ms.setJoints([rot, math.pi/4, math.pi/4, 0])

    time.sleep(0.03)
