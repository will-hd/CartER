"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from http://incompleteideas.net/sutton/book/code/pole.c
permalink: https://perma.cc/C9ZM-652R
"""

from time import sleep
from gym.envs.classic_control import rendering
import math
import gym
from gym import spaces, logger
from gym.utils import seeding
from matplotlib import test
import numpy as np

from scipy.integrate import odeint
from torch import positive

class CartPoleEnv(gym.Env):
    """
    DERIVED FROM GYM CARTPOLEENV.

    # Description:
    #     A pole is attached by an un-actuated joint to a cart, which moves along
    #     a frictionless track. The pendulum starts upright, and the goal is to
    #     prevent it from falling over by increasing and reducing the cart's
    #     velocity.

    # Source:
    #     This environment corresponds to the version of the cart-pole problem
    #     described by Barto, Sutton, and Anderson

    # Observation:
    #     Type: Box(4)
    #     Num     Observation               Min                     Max
    #     0       Cart Position             -4.8                    4.8
    #     1       Cart Velocity             -Inf                    Inf
    #     2       Pole Angle                -0.418 rad (-24 deg)    0.418 rad (24 deg)
    #     3       Pole Angular Velocity     -Inf                    Inf

    # Actions:
    #     Type: Discrete(2)
    #     Num   Action
    #     0     Push cart to the left
    #     1     Push cart to the right

    #     Note: The amount the velocity that is reduced or increased is not
    #     fixed; it depends on the angle the pole is pointing. This is because
    #     the center of gravity of the pole increases the amount of energy needed
    #     to move the cart underneath it

    # Reward:
    #     Reward is 1 for every step taken, including the termination step

    # Starting State:
    #     All observations are assigned a uniform random value in [-0.05..0.05]

    # Episode Termination:
    #     Pole Angle is more than 12 degrees.
    #     Cart Position is more than 2.4 (center of the cart reaches the edge of
    #     the display).
    #     Episode length is greater than 200.
    #     Solved Requirements:
    #     Considered solved when the average return is greater than or equal to
    #     195.0 over 100 consecutive trials.
    """

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        # physical constants
        self.gravity = 9.8 # a classic ...
        self.masspole = 10
        self.length = 3# actually half the pole's length

        self._DELTA_X_DOT_STEPS = 2000 # agents increment in speed at each action

        self.WORLD_TIME = 0
        self.dt = 0.02 # seconds between step updates
        self.tau = 0.0001  # seconds at which to iterate the ODE solving algorithm

        self.track_length = 10000 # steps (of stepper motor)

        self.rot_friction = 1 # currently unused

        # Angle at which to fail the episode
        self.THETA_FAIL_THRESHOLD_RADIANS = 12 * (2 * math.pi / 360) # 12 degrees

        # Characteristics of the RAIL:
        self.RAIL_STEP_LENGTH = 10000 #steps (of stepper for entire rail)
        self.RAIL_M_LENGTH = 1 #meter
        self.STEP_LENGTH = self.RAIL_M_LENGTH/self.RAIL_STEP_LENGTH #meters/step, for conversion to SI


        # Angle limit set to 2 * THETA_FAIL_THRESHOLD_RADIANS so failing observation
        # is still within bounds.
        high = np.array([self.RAIL_STEP_LENGTH * 2,
                        np.finfo(np.float32).max,
                        self.THETA_FAIL_THRESHOLD_RADIANS * 2,
                        np.finfo(np.float32).max],
                        dtype=np.float32)
        
        low = np.array([0,
                        -np.finfo(np.float32).max,
                        -self.THETA_FAIL_THRESHOLD_RADIANS * 2,
                        -np.finfo(np.float32).max],
                        dtype=np.float32)
        print(low, high)
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        self.MAX_STEPS = 10000
        self.step_count = 0 #

        self.seed()
        self.viewer = None
        self.state = None

        self.steps_beyond_done = None

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    @staticmethod
    def pole_dynamics(y, t, x_ddot, MASSPOLE, LENGTH, GRAVITY, STEP_LENGTH):
        """
        Equations of motion for cart
        """
        theta, theta_dot = y

        x_ddot_in_meters = x_ddot * STEP_LENGTH
        # print(x_ddot_in_meters)
        # For the interested reader:
        # https://coneural.org/florian/papers/05_cart_pole.pdf
        # Eq (14):*******minus sign
        # Theta zero at top and +ve going clockwise from vertical
        thetaacc = 3.0 / (4.0 * MASSPOLE * LENGTH**2) * (
            MASSPOLE*LENGTH * (GRAVITY*math.sin(theta) - x_ddot_in_meters*math.cos(theta)) 
        )
        dydt = [theta_dot, thetaacc]
        return dydt

    def step(self, action):
        """
        Computes a step of the environment, given the action from an agent.

        x, x_dot, x_ddot are all in units of in steps (s-1 s-2), it is the job of pole_dynamics() to convert to relevant
        units (meters s-1 s-2) for accurate dynamics...
        """
        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        _direction = 1 if action == 1 else -1 # left if action == 0, right if action == 1

        self.step_count += 1
        previous_world_time = self.WORLD_TIME # used for creating array of time steps
        self.WORLD_TIME += self.dt

        x_ddot = _direction * self._DELTA_X_DOT_STEPS / self.dt #acceleration (in steps s-2)

        # Get current state
        x_steps, x_dot_steps, theta, theta_dot = self.state
        
        # Calculate next state
        x_dot_steps = x_dot_steps + _direction * self._DELTA_X_DOT_STEPS #steps s-1
        x_steps = x_steps + self.dt * x_dot_steps #steps

        time_points = np.linspace(previous_world_time, self.WORLD_TIME, 1001) # array of time steps for ODE alg
        new_angles = odeint(self.pole_dynamics, (theta, theta_dot), time_points, 
                        args=(x_ddot, self.masspole, self.length, self.gravity, self.STEP_LENGTH))

        # positive_angle = (2*np.pi + new_angles[-1, 0])%(2*np.pi)
        self.state = (round(x_steps, 0), x_dot_steps, new_angles[-1, 0], new_angles[-1, 1])
        
        # print(f"Time: {self.WORLD_TIME}")
        # print(self.state)

        # Check if any failure conditions met
        done = bool(
            x_steps < 0
            or x_steps > self.RAIL_STEP_LENGTH
            or theta < -self.THETA_FAIL_THRESHOLD_RADIANS
            or theta > self.THETA_FAIL_THRESHOLD_RADIANS
            or self.step_count > self.MAX_STEPS
        )
        
        print(self.state)
        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0
        
        # sleep(0.05)
        return np.array(self.state), reward, done, {}

    def reset(self):
        self.state = [self.RAIL_STEP_LENGTH/2, 0, 0, 0] # starts in centre of rail with pole down
        self.steps_beyond_done = None
        self.step_count = 0
        self.WORLD_TIME = 0
        return np.array(self.state)

    def render(self, mode='human'):
        screen_width = 600
        screen_height = 600

        world_width = self.RAIL_M_LENGTH* 5
        self.SCALE = screen_width/world_width

        carty = screen_height/3 # TOP OF CART
        polewidth = 0.02*self.SCALE
        polelen = self.length*self.SCALE
        cartwidth = 0.1*self.SCALE
        cartheight = 0.06*self.SCALE

        if self.viewer is None:

            self.viewer = rendering.Viewer(screen_width, screen_height)
            l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
            axleoffset = cartheight / 4.0
            cart = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            self.carttrans = rendering.Transform()
            cart.add_attr(self.carttrans)
            self.viewer.add_geom(cart)
            l, r, t, b = -polewidth / 2, polewidth / 2, polelen - polewidth / 2, -polewidth / 2
            pole = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            pole.set_color(.8, .6, .4)
            self.poletrans = rendering.Transform(translation=(0, axleoffset))
            pole.add_attr(self.poletrans)
            pole.add_attr(self.carttrans)
            self.viewer.add_geom(pole)
            self.axle = rendering.make_circle(polewidth/2)
            self.axle.add_attr(self.poletrans)
            self.axle.add_attr(self.carttrans)
            self.axle.set_color(.5, .5, .8)
            self.viewer.add_geom(self.axle)
            self.track = rendering.Line((0, carty), (screen_width, carty))
            self.track.set_color(0, 0, 0)
            self.viewer.add_geom(self.track)

            self.limit_line_right = rendering.Line((self.RAIL_M_LENGTH * self.SCALE/2 + screen_width /2, carty/2), 
                                              (self.RAIL_M_LENGTH * self.SCALE/2 + screen_width /2, carty+carty/2))
            self.viewer.add_geom(self.limit_line_right)
            self.limit_line_left = rendering.Line((-self.RAIL_M_LENGTH * self.SCALE/2 + screen_width /2, carty/2), 
                                             (-self.RAIL_M_LENGTH * self.SCALE/2 + screen_width /2, carty+carty/2))
            self.viewer.add_geom(self.limit_line_left)

            self._pole_geom = pole

        if self.state is None:
            return None

        # Edit the pole polygon vertex
        pole = self._pole_geom
        l, r, t, b = -polewidth / 2, polewidth / 2, polelen - polewidth / 2, -polewidth / 2
        pole.v = [(l, b), (l, t), (r, t), (r, b)]

        x_meters = self.state[0] * self.STEP_LENGTH
        # print(f"x in meters: {x_meters}, x in steps {self.state[0]}")
        angle = self.state[2]
        cartx = x_meters * self.SCALE + (-self.RAIL_M_LENGTH * self.SCALE/2 + screen_width /2) # MIDDLE OF CART
        self.carttrans.set_translation(cartx, carty)
        self.poletrans.set_rotation(-angle)

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None


# test_cart = CartPoleEnv()
# test_cart.reset()

# for _ in range(8):
#     test_cart.step(1)
#     print(test_cart.state)
#     test_cart.render()

# for _ in range(50):
#     test_cart.step(0)
#     print(test_cart.state)
#     test_cart.render()

# for _ in range(25):
#     test_cart.step(0)
#     test_cart.render()