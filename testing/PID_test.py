from simulation1 import CartPoleEnv
from matplotlib import pyplot as plt
env = CartPoleEnv()
observation = env.reset()

Kp = 135 # 135
Ki = 96.5 # 96.5
Kd = 107.5 # 47.5

action = 0
integral = 0
for _ in range(1000):
  env.render()

  observation, reward, done, info = env.step(action)

  velocity = observation[1]
  angle = observation[2]
  angular_velocity = observation[3]

  integral = integral + angle

  F = Kp*(angle) + Kd*(angular_velocity) + Ki*(integral)

  action = 1 if F > 0 else 0
  if done:
    observation = env.reset()
    integral = 0
env.close()