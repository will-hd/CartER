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

centre_position = observation[0]
print(centre_position)

# 235, 0, 17.5 works!!!

Kp = 235 # 135
Ki = 0.1 # 226.5 # 96.5
Kd = 17.5 # 47.5
K_position_p = 0#-3 #+0.0002
K_position_i = 0#0.004 #0.02#+0.002
K_position_d = 0# +0.002

action = 0
integral = 0
integral2 = 0

ANGLE_OFFSET = 2050
INIT_OFFSET = 2052 # roughly the offset that keeps cart central on track
OFFSET_THRESHOLD = 5

def adjust_angle_offset(position, ANGLE_OFFSET):
    if position > 0:
        if (ANGLE_OFFSET > INIT_OFFSET - OFFSET_THRESHOLD):
            ANGLE_OFFSET -= 1
    
    elif position < 0:
        if ANGLE_OFFSET < INIT_OFFSET + OFFSET_THRESHOLD:
            ANGLE_OFFSET += 1
    return ANGLE_OFFSET

for _ in range(100000):
#   env.render()

    observation, reward, done, info = env.step(action)
    print(observation)
    position = observation[0] - centre_position
    velocity = observation[1]
    ANGLE_OFFSET = adjust_angle_offset(position, ANGLE_OFFSET)
    print(f"OFFSET: {ANGLE_OFFSET}")
    angle = observation[2] - ANGLE_OFFSET # offsets from 2048 help keep cart in the middle (wihtout knowledge of position)
    angular_velocity = observation[3]
    # print(f"State: {angle}, {angular_velocity}")

    integral = integral + angle
    integral2 = integral2 + position

    pid_angle = Kp*(angle) + Kd*(angular_velocity) + Ki*(integral)
    pid_position = K_position_p*(position) + K_position_d*(velocity) + K_position_i*(integral2)
    print(pid_angle)
    print(pid_position)
    F = pid_angle + pid_position

    action = 1 if F > 0 else 0
    print(f"action: {action}")
    if done:
        observation = env.reset()
        integral = 0
        integral2 = 0