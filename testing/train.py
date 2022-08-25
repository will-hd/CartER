from simulation_BOUNCE import CartPoleEnv
# import gym.envs.classic_control.cartpole
import numpy as np 
import matplotlib.pyplot as plt
import time

# env = gym.envs.classic_control.cartpole.CartPoleEnv()
env = CartPoleEnv()
obs = env.reset()

print(env.observation_space.low,"\n",env.observation_space.high)

def Qtable(state_space,action_space,bin_size = 30):
    
    bins = [np.linspace(-4.8,4.8,bin_size),
            np.linspace(-4,4,bin_size),
            np.linspace(-0.418,0.418,bin_size),
            np.linspace(-4,4,bin_size)]
    
    # q_table = np.reshape(np.loadtxt('testing/q_table_init.txt'), ([bin_size] * state_space + [action_space]))
    q_table = np.random.uniform(low=-1,high=1,size=([bin_size] * state_space + [action_space]))
    # temp = q_table.flatten()

    # np.savetxt('q_table_init.txt', temp)

    # print(q_table.shape)
    return q_table, bins


# Qtable(2,2)
def Discrete(state, bins):
    index = []
    for i in range(len(state)): 
        index.append(np.digitize(state[i],bins[i]) - 1)
    return tuple(index)

render = True

def Q_learning(q_table, bins, episodes = 5000, gamma = 0.95, lr = 0.1, timestep = 5000, epsilon = 0.2):
    rewards = 0
    solved = False 
    steps = 0 
    runs = [0]
    data = {'max' : [0], 'avg' : [0]}
    start = time.time()
    ep = [i for i in range(0,episodes + 1,timestep)] 
    
    for episode in range(1,episodes+1):
        
        current_state = Discrete(env.reset(),bins) # initial observation
        score = 0
        done = False
        temp_start = time.time()
        
        while not done:
            steps += 1 
            ep_start = time.time()

            if render:
                if episode%timestep == 0:
                    env.render()
            
            # epsilon greedy action
            if np.random.uniform(0,1) < epsilon:
                action = env.action_space.sample()
            
            else:
                action = np.argmax(q_table[current_state])
            
            observation, reward, done, info = env.step(action)
            next_state = Discrete(observation,bins)

            score += reward
            

            if not done:
                max_future_q = np.max(q_table[next_state])
                current_q = q_table[current_state+(action,)]
                new_q = (1-lr)*current_q + lr*(reward + gamma*max_future_q)
                q_table[current_state+(action,)] = new_q

            current_state = next_state
            
        # End of the loop update
        else:
            rewards += score
            runs.append(score)
            if score > 195 and steps >= 100 and solved == False: # considered as a solved:
                solved = True
                print('Solved in episode : {} in time {}'.format(episode, (time.time()-ep_start)))
        
        # Timestep value update
        if episode%timestep == 0:
            print('Episode : {} | Reward -> {} | Max reward : {} | Time : {}'.format(episode,rewards/timestep, max(runs), time.time() - ep_start))
            data['max'].append(max(runs))
            data['avg'].append(rewards/timestep)
            if rewards/timestep >= 195: 
                print('Solved in episode : {}'.format(episode))
            rewards, runs= 0, [0] 
            
    if len(ep) == len(data['max']):
        plt.plot(ep, data['max'], label = 'Max')
        plt.plot(ep, data['avg'], label = 'Avg')
        plt.xlabel('Episode')
        plt.ylabel('Reward')
        plt.legend(loc = "upper left")
        
    env.close()

# TRANING
q_table, bins = Qtable(len(env.observation_space.low), env.action_space.n)


Q_learning(q_table, bins, lr = 0.15, gamma = 0.995, episodes = 10*10**3, timestep = 1000)

