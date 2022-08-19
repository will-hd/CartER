import numpy as np
import time
import matplotlib.pyplot as plt
import random

import stable_baselines3
from commander.ml.environment import ExperimentalCartpoleEnv
from commander.log import setup_logging

setup_logging(debug=True, file=False)
env = ExperimentalCartpoleEnv()

class Q:

    def __init__(self, env) -> None:
        self.env = env
        self.obs = env.reset()

    def Qtable(self, state_space, action_space, bin_size = 30):
        """
        Initialises q-table, with given bin size.
        """
        bins = [np.linspace(0,25000,bin_size),
                np.linspace(-5000,5000,bin_size)]

        q_table = np.random.uniform(low=-1,high=1,size=([bin_size] * state_space + [action_space]))
        return q_table, bins

    def Discretise(self, state, bins):
        """
        Returns index of the bins that the state is in.
        """
        index = []
        for i in range(len(state)): 
            index.append(np.digitize(state[i],bins[i]) - 1)
        return tuple(index)

    render = True

    def Q_learning(self, q_table, bins, episodes = 1000, gamma = 0.95, lr = 0.1, timestep = 5000, epsilon = 0.7):
        
        epsilon0 = epsilon
        rewardscore = 0
        solved = False 
        steps = 0 
        runs = [0]
        data = {'max' : [0], 'avg' : [0]}
        start = time.time()
        ep = [i for i in range(0,episodes + 1,timestep)]
        
        for episode in range(1,episodes+1):
            steps = 0
            obs_init = self.env.reset()
            current_state = self.Discretise(obs_init,bins)
            print(obs_init)
            score = 0
            done = False
            temp_start = time.time()
            
            while not done:
                steps += 1 
                ep_start = time.time()
                
                # if True:
                #     if episode%(1*timestep) == 0:
                #         self.render()
                
                # epsilon greedy action
                # 1 is right, 0 is left
                if np.random.uniform(0,1) < epsilon:
                    action = random.choice([0,1])
                else:
                    action = np.argmax(q_table[current_state])

                observation, reward, done, infos = self.env.step(action)
                if steps == 1 or steps == 2:
                    print(f"Init obs = {observation}")
                # Unpack dictionaries since only using one cart
                
                next_state = self.Discretise(observation,bins)

                score += reward
                

                if not done:
                    max_future_q = np.max(q_table[next_state])
                    current_q = q_table[current_state+(action,)]
                    new_q = (1-lr)*current_q + lr*(reward + gamma*max_future_q)
                    q_table[current_state+(action,)] = new_q

                current_state = next_state
            

            
            rewardscore += score
            runs.append(score)
        
            # reduce epsilon
            epsilon -= epsilon0/episodes
            if episode%timestep == 0:
                print(f"epsilon = {epsilon}")

            # Timestep value update
            if episode%timestep == 0:
                print('Episode : {} | Reward -> {} | Max reward : {} | Time : {}'.format(episode,rewardscore/timestep, max(runs), time.time() - ep_start))
                data['max'].append(max(runs))
                data['avg'].append(rewardscore/timestep)
                if rewardscore/timestep >= 960: 
                    print('Solved in episode : {}'.format(episode))
                
                with open("q_learning_results.txt" ,"a") as log:
                    log.write("\n Episode : {} | Reward -> {} | Max reward : {} | Time : {}'".format(episode,rewardscore/timestep, 
                    max(runs), time.time() - ep_start))

                rewardscore, runs= 0, [0] 
                
                
        if len(ep) == len(data['max']):
            plt.plot(ep, data['max'], label = 'Max')
            plt.plot(ep, data['avg'], label = 'Avg')
            plt.xlabel('Episode')
            plt.ylabel('Reward')
            plt.legend(loc = "upper left")
            plt.show()
            
        
    
    def Train(self):
        self.q_table, self.bins = self.Qtable(state_space=2, action_space=2)
        print(self.q_table.shape)
        print(self.bins)
        self.Q_learning(self.q_table, self.bins, lr = 0.15, gamma = 0.995, episodes = 300, timestep = 2)

test = Q(env)
test.Train()
