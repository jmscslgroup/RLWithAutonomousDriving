import random
import numpy as np
from keras.optimizers import Adam
from keras.models import Sequential      # One layer after the other
from keras.layers import Dense		 # Dense layers are fully connected layers, Flatten layers flatten out multidimensional inputs
from collections import deque            # For storing moves 

class QLearn:
    def __init__(self,gamma,LEARNING_RATE,MEMORY_SIZE,EXPLORATION_MAX,EXPLORATION_MIN,EXPLORATION_DECAY,BATCH_SIZE,  observation_space, action_space):
        
        self.gamma = gamma
        self.exploration_rate = EXPLORATION_MAX
        self.LEARNING_RATE = LEARNING_RATE
        self.MEMORY_SIZE = MEMORY_SIZE
        self.EXPLORATION_MAX = EXPLORATION_MAX
        self.EXPLORATION_MIN = EXPLORATION_MIN
        self.EXPLORATION_DECAY = EXPLORATION_DECAY
        self.BATCH_SIZE = BATCH_SIZE

        self.action_space = action_space
        self.memory = deque(maxlen=MEMORY_SIZE)
	
	#Create NN model
        self.model = Sequential()
        self.model.add(Dense(24, input_shape=(observation_space,), activation="relu"))
        self.model.add(Dense(24, activation="relu"))
        self.model.add(Dense(self.action_space, activation="linear"))
        self.model.compile(loss="mse", optimizer=Adam(lr=LEARNING_RATE))
        
    # Store data to memory deque
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    # Decide to explore or exploit the environment
    def act(self, state):
        if np.random.rand() < self.exploration_rate:
            return random.randrange(self.action_space) 			#explore
        q_values = self.model.predict(state)				#exploit: input the state to model ,and the model outputs the predicted values for each action 
        return np.argmax(q_values[0])					#take the max value

    def experience_replay(self):
        if len(self.memory) < self.BATCH_SIZE:
            return
        batch = random.sample(self.memory, self.BATCH_SIZE)        	# take a random batch from memory
        for state, action, reward, state_next, terminal in batch:  	# Calculate reward
            q_update = reward
            if not terminal:
                q_update = (reward + self.gamma * np.amax(self.model.predict(state_next)[0]))
            q_values = self.model.predict(state)
            q_values[0][action] = q_update
            self.model.fit(state, q_values, verbose=0)
        self.exploration_rate *= self.EXPLORATION_DECAY		   	# decay the exploration_rate
        self.exploration_rate = max(self.EXPLORATION_MIN, self.exploration_rate)

    def save(self):
	    self.model.save('/home/nguy2539/catvehicle_ws/trained_nn.h5')
	    print "model saved"


'''
    
    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def learnQ(self, state, action, reward, value):
        
        oldv = self.q.get((state, action), None)
        if oldv is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = oldv + self.alpha * (value - oldv)

    def chooseAction(self, state, return_q=False):
        q = [self.getQ(state, a) for a in self.actions]
        maxQ = max(q)

        if random.random() < self.epsilon:
            minQ = min(q); mag = max(abs(minQ), abs(maxQ))
            # add random values to all the actions, recalculate maxQ
            q = [q[i] + random.random() * mag - .5 * mag for i in range(len(self.actions))] 
            maxQ = max(q)

        count = q.count(maxQ)
        # In case there're several state-action max values 
        # we select a random one among them
        if count > 1:
            best = [i for i in range(len(self.actions)) if q[i] == maxQ]
            i = random.choice(best)
        else:
            i = q.index(maxQ)

        action = self.actions[i]        
        if return_q: # if they want it, give it!
            return action, q
        return action

    def learn(self, state1, action1, reward, state2):
        maxqnew = max([self.getQ(state2, a) for a in self.actions])
        self.learnQ(state1, action1, reward, reward + self.gamma*maxqnew)
'''
