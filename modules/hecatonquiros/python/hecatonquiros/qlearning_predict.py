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
import sys
import time

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
model.load_weights(sys.argv[1])


while True:
    print("Enter coordinates: ")
    x = float(raw_input("x: "))
    y = float(raw_input("y: "))
    z = float(raw_input("x: "))
    raw_input("Press Enter to start...")

    env.reset()
    env.setTarget(np.array([x,y,z]))
    ob = env.observe()
    for i in range(50):
        ob = ob.reshape(1,1,7)
        action = model.predict(ob)
        a = np.argmax(action)
        ob, rew, op, _ = env.step(a)
        print(env.ms_.jointsTransform()[-1][0:3,3])
        time.sleep(0.03)
