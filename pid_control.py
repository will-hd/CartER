"""
Inspired by: https://ethanr2000.medium.com/using-pid-to-cheat-an-openai-challenge-f17745226449
"""
from commander.ml.environment import ExperimentalCartpoleEnv
from commander.log import setup_logging
from commander.network.protocol import SetVelocityPacket
from commander.network.network_constants import SetOperation

setup_logging(debug=False, file=False)
env = ExperimentalCartpoleEnv()
observation = env.reset()

centre_position = observation[0]

# 235, 0, 17.5 works!!!

# PID constants
K_angle_p = 235 # 135
K_angle_i = 0.1 # 226.5 # 96.5
K_angle_d = 47.5 # 47.5

K_position_p = 0.5 #-3 #+0.0002
K_position_i = 0 #0.004 #0.02#+0.002
K_position_d = 0.1 # +0.002

action = 0
integral_angle = 0
integral_position = 0

# def adjust_angle_offset(position, velocity, ANGLE_OFFSET):
#     if position > 0 and velocity > 2000:
#         ANGLE_OFFSET = 2051
    
#     elif position < 0 and velocity < -2000:
#         ANGLE_OFFSET = 2052


#     return ANGLE_OFFSET

def switch_action(action):
    if action == 0:
        return 1
    elif action == 1:
        return 0
    else:
        print("Invalid action")
start = False

while not start:
    observation, reward, done, info = env.step(action)
    action = switch_action(action)
    print(observation)
    position = observation[0] - centre_position
    velocity = observation[1]
    angle = observation[2] - 2051.5 # 2051/2052offsets from ~2048 help keep cart in the middle (wihtout knowledge of position) 
    # not exactly 2048 perhaps due to weights being offcentre
    angular_velocity = observation[3]

    if abs(angular_velocity) < 60 and abs(angle) < 50:
        start = True
        break


for _ in range(100000):
#   env.render()

    observation, reward, done, info = env.step(action)
    print(observation)
    position = observation[0] - centre_position
    velocity = observation[1]
    angle = observation[2] - 2051.5 # 2051/2052offsets from ~2048 help keep cart in the middle (wihtout knowledge of position) 
    # not exactly 2048 perhaps due to weights being offcentre
    angular_velocity = observation[3]

    integral_angle = integral_angle + angle
    integral_position = integral_position + position

    pid_angle = K_angle_p*(angle) + K_angle_d*(angular_velocity) + K_angle_i*(integral_angle)
    pid_position = K_position_p*(position) + K_position_d*(velocity) + K_position_i*(integral_position)

    F = pid_angle + pid_position

    action = 1 if F > 0 else 0
    print(f"action: {action}")

    if done:
        observation = env.reset()
        integral_angle = 0
        integral_position = 0