
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

        self.action_space = gym.spaces.Box(	low=np.array([-0.01, -0.01, -0.01, -0.01]), 
					high=np.array([0.01, 0.01, 0.01, 0.01]), 
					dtype=np.float16)
        self.observation_space = gym.spaces.Box(low=math.pi/2, high=math.pi/2, shape=(4, 1), dtype=np.float16)

    def step(self, action):
        cj = np.squeeze(np.array(self.ms_.getJoints()) + np.array(action))
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
        
        return ob, reward, episode_over

    def reset(self):
        self.ms_.setJoints([0,0,0,0])
        self.currentStep = 0


if __name__ == "__main__":
    armEnv = ArmEnv([0,-0.1,0.1])

    while(True):
        action = armEnv.action_space.sample()
        ob, reward, eo = armEnv.step(action)
        time.sleep(0.01)
        if(eo):
            armEnv.reset()

