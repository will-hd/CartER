import numpy as np
import matplotlib.pyplot as plt

def Qtable(state_space, action_space, bin_size = 40):
        """
        Initialises q-table, with given bin size.
        """
        x = np.linspace(0,25000,bin_size)
        y = np.linspace(-5000,5000,bin_size)



        bins = [x,
                y]

        q_table = np.random.uniform(low=-1,high=1,size=([bin_size] * state_space + [action_space]))
        return q_table, bins

T1 = np.linspace(0, 5, 20)
T2 = np.linspace(5, 10, 20)
a = np.unique([np.sqrt(1 - (0.2*T1-1)**2), np.sqrt(1 - (0.2*T2-1)**2)])
print(a)