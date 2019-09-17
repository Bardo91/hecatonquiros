import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

import numpy as np 

from ArmEnv import ArmEnv

# Create the environment
env = ArmEnv([-0.2,0,0.2], _fixTarget = True)
#np.random.seed(123)
#env.seed(123)
env.reset()
env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run

# Define the model
model = PPO2(MlpPolicy, env, verbose=1, tensorboard_log="./ppo_bipedal_tensorboard/")

# Train the agent
model.learn(total_timesteps=100000)

while True:
    input("Press key to see what happens")
    # After training, watch our agent walk
    obs = env.reset()
    for i in range(1000):
        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        env.render()
        if done:
            break
