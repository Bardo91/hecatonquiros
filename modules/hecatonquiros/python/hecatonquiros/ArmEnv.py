
import IPython

from ModelSolverOR import ModelSolverOR

from openravepy import *
import numpy as np
import time
import math
import gym
from gym import spaces

class ArmEnv(gym.Env):
    metadata = {'render.modes': ['robot']}

    def __init__(self, _targetPosition):
        self.data_ = {}
        self.data_["visualize"] = True
        self.data_["robot_file"]= "/home/bardo91/programming/hecatonquiros/modules/hecatonquiros/config/arm_4dof.robot.xml"
        self.data_["enable_physics"] = True
        self.ms_ = ModelSolverOR(self.data_)
        self.ms_.orEnv_.StopSimulation()  # Stop simulation to control step by step

        self.targetPosition_ = np.array(_targetPosition)
        self.currentStep = 0

        self.action_space = gym.spaces.Discrete(4)
        self.observation_space = gym.spaces.Box(low=math.pi/2, high=math.pi/2, shape=(4, 1), dtype=np.float16)

    def step(self, action):
        a_val = [0,0,0,0]
        for i in range(4):
            if(action[i] == 0):
                a_val[i] = -0.1
            if(action[i] == 1):
                a_val[i] = 0.1
            if(action[i] == 2):
                a_val[i] = 0.1
                
        cj = np.squeeze(np.array(self.ms_.getJoints())+np.array(a_val))
        self.ms_.setJoints(cj)
        self.ms_.orEnv_.StepSimulation(0.03)

        distToTarget = np.linalg.norm(    self.targetPosition_ - 
                                    self.ms_.jointsTransform()[-1][0:3,3])

        reward = 1
        self.currentStep +=1

        ob = self.ms_.getJoints()
        
        episode_over = False
        if(self.currentStep > 500 or distToTarget > 0.3):
            episode_over = True        
        
        return ob, reward, episode_over, ""

    def reset(self):
        self.ms_.setJoints([0,0,0,0])
        self.currentStep = 0
        return self.ms_.getJoints()

