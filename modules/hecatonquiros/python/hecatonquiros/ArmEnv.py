
import IPython

from ModelSolverOR import ModelSolverOR

from openravepy import *
import numpy as np
import time
import math
import gym
from gym import spaces
import itertools

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

        self.seed_ = 0
        self.possibleActions_ = [0,1, 2, 3]
        self.completeActionSpace_ = [p for p in itertools.product(self.possibleActions_, repeat=4)]

        self.action_space = gym.spaces.Discrete(len(self.completeActionSpace_))
        # self.observation_space = gym.spaces.Box(low=math.pi/2, high=math.pi/2, shape=(4,), dtype=np.float16)
        self.observation_space = gym.spaces.Box(low=np.array([-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -0.2, -0.2, 0.05]), 
                                                high=np.array([math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.2, 0.2, 0.2]), dtype=np.float32)
        self.targetPosition_ = self.observation_space.sample()[4:7]

    def step(self, action):
        action_list = self.completeActionSpace_[action]
        a_val = [0.0,0.0,0.0,0.0]
        for i in range(len(action_list)):
            if(action_list[i] == 0):
                a_val[i] = -0.05
            if(action_list[i] == 1):
                a_val[i] = 0
            if(action_list[i] == 2):
                a_val[i] = 0.05


        cj = np.squeeze(np.array(self.ms_.getJoints())+np.array(a_val))
        self.ms_.setJoints(cj)
        self.ms_.orEnv_.StepSimulation(0.03)

        distToTarget = np.linalg.norm(    self.targetPosition_ - 
                                    self.ms_.jointsTransform()[-1][0:3,3])

        reward = -1
        self.currentStep +=1

        ob = self.observe()
        
        episode_over = False
        if(self.currentStep > 500 or distToTarget > 0.3):
            episode_over = True   

        if(distToTarget < 0.01):
            episode_over = True     
        
        return ob, reward, episode_over, {}

    def setTarget(self, _target):
        self.targetPosition_ = _target
        if len(self.ms_.handles_) > 0:
            self.ms_.removeElement(0)
        self.ms_.drawPoint(self.targetPosition_)

    def reset(self):
        if len(self.ms_.handles_) > 0:
            self.ms_.removeElement(0)

        rs = [0,0,0,0] #np.random.random([4,1])*0.02 - 0.01
        self.ms_.setJoints(rs)
        self.currentStep = 0
        self.targetPosition_ = self.observation_space.sample()[4:7]
        self.ms_.drawPoint(self.targetPosition_)
        return self.observe()

    def observe(self):
        ob = np.array(self.ms_.getJoints()).reshape(4,)
        ob = np.append(ob, self.targetPosition_)
        return ob

    def seed(self, _seed):
        self.seed_ = _seed

    def render(self, mode='arm', close=False):
        # time.sleep(0.03)
        pass