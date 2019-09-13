import numpy as np
import gym

import IPython

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import EpsGreedyQPolicy
from rl.memory import SequentialMemory

from ArmEnv import ArmEnv

import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--iters', type=int, help='Iterations for training', default = 50000)
parser.add_argument('--model', type=str, help='Path to model', default="sample_train.hdf5")
parser.add_argument('--model_in', type=str, help='Path to model')
args = parser.parse_args()

# Get the environment and extract the number of actions available in the Cartpole problem
env = ArmEnv([-1.64651804e-01, 0,  1.84651804e-01])
np.random.seed(123)
env.seed(123)
env.reset()
nb_actions = env.action_space.n

model = Sequential()
model.add(Flatten(input_shape=(1,) + env.observation_space.shape))
model.add(Dense(16, kernel_initializer='glorot_uniform', bias_initializer='zeros'))
model.add(Activation('relu'))
model.add(Dense(32, kernel_initializer='glorot_uniform', bias_initializer='zeros'))
model.add(Activation('relu'))
model.add(Dense(64, kernel_initializer='glorot_uniform', bias_initializer='zeros'))
model.add(Activation('relu'))
model.add(Dense(128, kernel_initializer='glorot_uniform', bias_initializer='zeros'))
model.add(Activation('relu'))
model.add(Dense(128, kernel_initializer='glorot_uniform', bias_initializer='zeros'))
model.add(Activation('relu'))
model.add(Dense(nb_actions))
model.add(Activation('linear'))
print(model.summary())

if args.model_in not None:
    model.load_weights(args.model_in)

policy = EpsGreedyQPolicy()
memory = SequentialMemory(limit=50000, window_length=1)
dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10, target_model_update=1e-2, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])
# Okay, now it's time to learn something! We visualize the training here for show, but this slows down training quite a lot. 
dqn.fit(env, nb_steps=args.iters, visualize=True, verbose=2)

dqn.test(env, nb_episodes=5, visualize=True)
model.save_weights(args.model)