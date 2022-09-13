"""
PID from automated swingup DEMO:

Uses similar syntax to gym API. 
Running this file should cause the cart to do a limit finding routine (from within env.reset()).
Then it will swing the pendulum until it can complete a loop, stop the cart, and then let the 
pendulum perform complete loops until it does one that is slow enough for the PID control to 
take over/"catch the cart" at the top.  

PID inspired from: https://ethanr2000.medium.com/using-pid-to-cheat-an-openai-challenge-f17745226449
"""
from commander.ml.environment import ExperimentalCartpoleEnv
from commander.log import setup_logging
from commander.network.protocol import SetVelocityPacket
from commander.network.network_constants import SetOperation

setup_logging(debug=False, file=False)
env = ExperimentalCartpoleEnv()
observation = env.reset()

centre_position = observation[0]

# PID constants
K_angle_p = 235 # 235
K_angle_i = 0.1 # 0.1
K_angle_d = 47.5 # 47.5

K_position_p = 0.5 #0.5
K_position_i = 0
K_position_d = 0.1 # +0.1

action = 0
integral_angle = 0
integral_position = 0

def switch_action(action):
    """
    Switches the action between left <-> right.
    Keeps the cart roughly central until swingup complete.
    """
    if action == 0:
        return 1                
    elif action == 1:
        return 0
    else:
        print("Invalid action")

start_flag = False
swingup_flag = False

# 2051/2052 offsets from ~2048 help keep cart in the middle (wihtout knowledge of position) 
# not exactly 2048 (middle of 0-4095 for 12 bit encoder) perhaps due to weights on pendulum being offcentre
# this makes angle zeroed vertically upwards
ANGLEOFFSET = 2051.5 

# Perform a swingup sequenece to give enough energy to complete loop
for _ in range(100000):
    observation, reward, done, info = env.step(action)

    position = observation[0] - centre_position
    velocity = observation[1]
    angle = observation[2] - ANGLEOFFSET
    angular_velocity = observation[3]

    # Swingup "logic"
    if angular_velocity > 0 and abs(angle) > 1000:
        action = 1
    elif angular_velocity < 0 and abs(angle) > 1000:
        action = 0

    # If angle goes through zero at top
    if abs(angle) < 100:
        swingup_flag = True

    # Stop swing up and cart
    if swingup_flag:
        velo_pkt = SetVelocityPacket(SetOperation.EQUAL, cart_id=env._agent.cart_id, value=0, actobs_tracker=22) # no need for cart_id ...
        env.network_manager.send_packet(velo_pkt)
        break

# Wait for pendulum to be vertical with little velocity.
# In previous tests, the pendulum makes 
while not start_flag:
    observation, reward, done, info = env.step(action)
    action = switch_action(action)

    position = observation[0] - centre_position
    velocity = observation[1]
    angle = observation[2] - ANGLEOFFSET
    angular_velocity = observation[3]

    # Wait for pendulum to be slow enough 
    if abs(angular_velocity) < 60 and abs(angle) < 50:
        start_flag = True
        break # Switch to PID

# PID
for _ in range(100000):
    observation, reward, done, info = env.step(action)

    position = observation[0] - centre_position
    velocity = observation[1]
    angle = observation[2] - ANGLEOFFSET 
    angular_velocity = observation[3]

    integral_angle = integral_angle + angle
    integral_position = integral_position + position

    pid_angle = K_angle_p*(angle) + K_angle_d*(angular_velocity) + K_angle_i*(integral_angle)
    pid_position = K_position_p*(position) + K_position_d*(velocity) + K_position_i*(integral_position)

    F = pid_angle + pid_position
    action = 1 if F > 0 else 0 # Next action

    if done:
        observation = env.reset()
        integral_angle = 0
        integral_position = 0