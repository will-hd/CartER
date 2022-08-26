from simulation import CartPoleEnv
# import gym.envs.classic_control.cartpole
import numpy as np 
import matplotlib.pyplot as plt
import time

# env = gym.envs.classic_control.cartpole.CartPoleEnv()
env = CartPoleEnv()
obs = env.reset()

print(env.observation_space.low,"\n",env.observation_space.high)

def Qtable(state_space,action_space,bin_size = 30):
    """
    Initialises q-table, with given bin size.
    """
    bins = [np.linspace(0,10000,bin_size),
            np.linspace(-1000, 1000 ,bin_size),
            np.linspace(-2.4, 2.4, bin_size),
            np.linspace(-100, 100, bin_size)]
            
    
    # q_table = np.reshape(np.loadtxt('testing/q_table_init.txt'), ([bin_size] * state_space + [action_space]))
    q_table = np.random.uniform(low=-1,high=1,size=([bin_size] * state_space + [action_space]))
    # temp = q_table.flatten()

    # np.savetxt('q_table_init.txt', temp)

    # print(q_table.shape)
    return q_table, bins


# Qtable(2,2)
def Discrete(state, bins):
    """
    Returns index of the bins that the state is in.
    """
    index = []
    for i in range(len(state)): 
        index.append(np.digitize(state[i],bins[i]) - 1)
    return tuple(index)

render = True

def Q_learning(q_table, bins, episodes = 5000, gamma = 0.95, lr = 0.1, timestep = 5000, epsilon = 0.5):
    epsilon0 = epsilon
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
                if episode%(10*timestep) == 0:
                    env.render()
            
            # epsilon greedy action
            # 1 is right, 0 is left
            if np.random.uniform(0,1) < epsilon:
                action = env.action_space.sample()
            else:
                temp = q_table[current_state]
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
        if episode == 3000:
            pass    
        # End of the loop update
        else:
            rewards += score
            runs.append(score)
            if score > 960 and steps >= 100 and solved == False: # considered as a solved:
                solved = True
                print('Solved in episode : {} in time {}'.format(episode, (time.time()-ep_start)))

        # reduce epsilon
            epsilon -= epsilon0/episodes
            if episode%timestep == 0:
                print(f"epsilon = {epsilon}")

        # Timestep value update
        if episode%timestep == 0:
            print('Episode : {} | Reward -> {} | Max reward : {} | Time : {}'.format(episode,rewards/timestep, max(runs), time.time() - ep_start))
            data['max'].append(max(runs))
            data['avg'].append(rewards/timestep)
            if rewards/timestep >= 960: 
                print('Solved in episode : {}'.format(episode))
            rewards, runs= 0, [0] 
            
    if len(ep) == len(data['max']):
        plt.plot(ep, data['max'], label = 'Max')
        plt.plot(ep, data['avg'], label = 'Avg')
        plt.xlabel('Episode')
        plt.ylabel('Reward')
        plt.legend(loc = "upper left")
        # plt.show()
        
    env.close()

# TRANING
q_table, bins = Qtable(len(env.observation_space.low), env.action_space.n)
q_table_init = q_table.copy()
# 
Q_learning(q_table, bins, lr = 0.15, gamma = 0.995, episodes = 100, timestep = 500)



# plt.plot(q_table[:, 1, 1])
# plt.plot(q_table[:, 16, 1], label='1.1')
# plt.plot(q_table[:, 15, 1], label='2.1')
# plt.plot(q_table[:, 14, 1], label='3.1')
# plt.plot(q_table[:, 13, 1], label='4.1')
# plt.plot(q_table[:, 12, 1], label='5.1')

# plt.plot(q_table[16, :, 1], label='1.1')
# plt.plot(q_table[14, :, 1], label='2.1')
# plt.plot(q_table[15, :, 1], label='3.1')

# plt.plot(q_table[16, :, 0], label='1.0')
# plt.plot(q_table[14, :, 0], label='2.0')
# plt.plot(q_table[15, :, 0], label='3.0')
# plt.plot(q_table[:, 13, 0], label='4.0')
# plt.plot(q_table[:, 12, 0], label='5.0')

# # +ve favours right moves
# # slightly on the right, plot velocity
# plt.plot(q_table[16, :, 1]-q_table[16, :, 0], label='1')
# # slightly on the left, plot velocity
# plt.plot(q_table[13, :, 1]-q_table[13, :, 0], label='2')

# # moving right, plot position
# plt.plot(q_table[:, 16, 1]-q_table[:, 16, 0], label='3')
# # moving left, plot position
# plt.plot(q_table[:, 13, 1]-q_table[:, 13, 0], label='4')
# plt.plot(q_table[:, 12, 1]-q_table[:, 12, 0], label='5')
# plt.plot(q_table[:, 14, 1]-q_table[:, 14, 0], label='6')

# plt.legend()
# plt.show()