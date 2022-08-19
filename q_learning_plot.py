import matplotlib.pyplot as plt
import re
import numpy as np
from scipy.interpolate import make_interp_spline

with open("plot_results.txt", "r") as f:
    text = f.read()
    rewards_text = re.findall(r"(?<=->\s)\d{3,4}", text)
    rewards = [int(rew) for rew in rewards_text]

    max_rewards_text = re.findall(r"(?<=Max\sreward\s\:\s)\d{3,4}", text)
    max_rewards = [int(rew) for rew in max_rewards_text]




def mov_average(arr, window_size=2):
    i = 0
    # Initialize an empty list to store moving averages
    moving_averages = []

    while i < len(arr) - window_size + 1:
        window = rewards[i : i + window_size]
    
        # Calculate the average of current window
        window_average = round(sum(window) / window_size, 2)
        moving_averages.append(window_average)
        
        # Shift window to right by one position
        i += 1
    
    return moving_averages

# plt.plot(mov_average(rewards), label="avg. Smoothed")
plt.plot(rewards, label="avg. [raw]")
plt.plot(mov_average(rewards, window_size=6), label="avg. [smoothed]")


# plt.plot(mov_average(max_rewards), label="avg. maxSmoothed")
# plt.plot(max_rewards, label="avg. maxRaw")
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.legend()
plt.show()