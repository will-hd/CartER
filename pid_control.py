import numpy as np
import time
import matplotlib.pyplot as plt
import random

import stable_baselines3
from commander.ml.environment import ExperimentalCartpoleEnv
from commander.log import setup_logging

setup_logging(debug=False, file=False)
env = ExperimentalCartpoleEnv()
observation = env.reset()

# 235, 0, 17.5 works!!!

Kp = 235 # 135
Ki = 1 # 226.5 # 96.5
Kd = 17.5 # 47.5

action = 0
integral = 0
for _ in range(100000):
#   env.render()

  observation, reward, done, info = env.step(action)

  velocity = observation[1-1]
  angle = observation[2-1] - 2055 # offsets from 2048 help keep cart in the middle (wihtout knowledge of position)
  angular_velocity = observation[3-1]
  print(f"State: {angle}, {angular_velocity}")

  integral = integral + angle

  F = Kp*(angle) + Kd*(angular_velocity) + Ki*(integral)

  action = 1 if F > 0 else 0
  print(action)
  if done:
    observation = env.reset()
    integral = 0