#!/usr/bin/env python

import pickle
import yaml
import sys
import numpy as np
import time
import qlearn_hoang1
# ROS packages required
import rospy
import rospkg
import start_sim
import csv

# for any function that calls start_qlearning.py, this will be the main

#rospy.init_node('catvehicle_wall_qlearn', anonymous=True, log_level=rospy.WARN)


# Load the yaml file
with open('rl_params.yaml', 'r') as f:
    params = yaml.load(f)



# Set the logging system
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('catvehicle_openai_ros')
outdir = pkg_path + '/training_results'


nepisodes = params['catvehicle']['nepisodes']
nsteps = params['catvehicle']['nsteps']
gamma = params['catvehicle']['gamma']
LEARNING_RATE = params['catvehicle']['LEARNING_RATE']
MEMORY_SIZE = params['catvehicle']['MEMORY_SIZE']
EXPLORATION_MAX = params['catvehicle']['EXPLORATION_MAX']
EXPLORATION_MIN = params['catvehicle']['EXPLORATION_MIN']
EXPLORATION_DECAY = params['catvehicle']['EXPLORATION_DECAY']
BATCH_SIZE = params['catvehicle']['BATCH_SIZE']

# take observation space and action space as input for Neuron Network
observation_space = 2
action_space = 3

#initialize Deep Q learning neural network
deepQlearning = qlearn_hoang1.QLearn(gamma,LEARNING_RATE,MEMORY_SIZE,EXPLORATION_MAX,EXPLORATION_MIN,EXPLORATION_DECAY,BATCH_SIZE,  observation_space, action_space)
run = 0
scores = []
iterations = []

sim = start_sim.start_sim()

for x in range(nepisodes):
    # Spawn the Gazebo simulation
    
    sim.spawn()
    time.sleep(10) # Give simulation time to load
    
    run += 1
    state = sim._get_obs()
    state = np.reshape(state, [1, observation_space])
    reward_cumulative = 0
    rospy.logwarn('STARTING EPISODE: '+str(x))
    for i in range(nsteps):
        rospy.logwarn('ON ITERACTION: '+str(i))
        action = deepQlearning.act(state)                         			#decide action to take: explore or exploit
        state_next, reward, done, complete = sim.step(action)
        state_next = np.reshape(state_next, [1, observation_space])			#take next state
        deepQlearning.remember(state, action, reward, state_next, done)		#store next state
        state = state_next
        reward_cumulative += reward							#accumulate rewards
        if (i == (nsteps - 1)) and (done != True):
            done = True
            reward -= 1000
        if done:
            print "Run: " + str(run) + ", exploration: " + str(deepQlearning.exploration_rate) + ", score: " + str(reward_cumulative) + ", # Spots: ", str(complete), "/10"
            break
        deepQlearning.experience_replay()						#update reward using experience replay
    
    scores.append(reward_cumulative)
    iterations.append(x)

    sim.signal_handler(2)
    time.sleep(30)

    deepQlearning.save()
    pickle.dump(deepQlearning, open("picked_nn.p", "wb"))

with open('scores.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
    wr.writerow(scores)

with open('iterations.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
    wr.writerow(iterations)

# TODO: 1. Save Neural Network. 2. Replay Neural Network to exploit n-times. 3. Record the exploitation. 4. Publish loss function

# Saves NN to a file, which can be loaded using load_model.py
