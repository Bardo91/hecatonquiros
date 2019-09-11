import gym
import numpy as np # 1. Load Environment and Q-table structure
from ArmEnv import ArmEnv
import math
import IPython
armEnv = ArmEnv([0,0.1,0.1])
DIV_OBS = 32
OBS_STEP = math.pi/DIV_OBS
Q = np.zeros([DIV_OBS,DIV_OBS,DIV_OBS,DIV_OBS,armEnv.action_space.n,armEnv.action_space.n])

def obsToIndex(obs):
    a = [0,0,0,0]
    for i in range(4):
        val = obs[i] + math.pi/2
        idx = int(val/OBS_STEP)
        if idx > 31: 
            idx = 31
        a[i] = idx
    return a
# env.obeservation.n, env.action_space.n gives number of states and action in env loaded# 2. Parameters of Q-leanring
eta = .628
gma = .9
epis = 5000
rev_list = [] # rewards per episode calculate

# 3. Q-learning Algorithmfor i in range(epis):
for i in range(epis):
    # Reset environment
    s = armEnv.reset()
    rAll = 0
    d = False
    j = 0
    #The Q-Table learning algorithm
    while not d:
        j+=1
        # Choose action from Q table
        idxS = obsToIndex(s)
        # IPython.embed()
        action_table =  Q[idxS[0], idxS[1], idxS[2], idxS[3],:,:] +  np.array([np.random.randn(1,armEnv.action_space.n)*(1./(i+1)) for i in range(4)]).reshape(4,4)

        a = np.argmin(action_table, axis=0)
        #Get new state & reward from environment
        s1,r,d,_ = armEnv.step(a)
        #Update Q-Table with new knowledge
        idxS1 = obsToIndex(s1)
        Q[idxS[0], idxS[1], idxS[2], idxS[3],a] = Q[idxS[0], idxS[1], idxS[2], idxS[3],a] + eta*(r + gma*np.max(Q[idxS1[0], idxS1[1], idxS1[2], idxS1[3],:]) - Q[idxS[0], idxS[1], idxS[2], idxS[3],a])
        rAll += r
        s = s1
        if d == True:
            break
    print(rAll)
    rev_list.append(rAll)

print "Reward Sum on all episodes " + str(sum(rev_list)/epis)
print "Final Values Q-Table"
print Q