"""
Inspired by: https://ethanr2000.medium.com/using-pid-to-cheat-an-openai-challenge-f17745226449
"""
from curses import flash
from commander.ml.environment import ExperimentalCartpoleEnv
from commander.log import setup_logging

setup_logging(debug=False, file=False)
env = ExperimentalCartpoleEnv()
observation = env.reset()

centre_position = observation[0]
action = 1

for _ in range(100000):

    

    observation, reward, done, info = env.step(action)
    print(observation)
    position = observation[0] - centre_position
    velocity = observation[1]
    angle = observation[2] - 2051.5 # 2051/2052offsets from ~2048 help keep cart in the middle (wihtout knowledge of position) 
    # not exactly 2048 perhaps due to weights being offcentre
    angular_velocity = observation[3]

    
    if angular_velocity > 0:
        action = 1
    else:
        action = 0
#   env.render()
    