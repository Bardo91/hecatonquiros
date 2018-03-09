"""Loads up an environment, attaches a viewer, loads a scene, and requests information about the robot.
"""
from openravepy import *
import time
import IPython
from numpy import random, linalg

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)

with env: # lock the environment since robot will be used
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple

class Server:
    def init(self):


    # Possible request.
    # Query Joints.
    # set Joints.
    # Compute IK.
    def handleRequest(self):
        
