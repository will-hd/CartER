
"""
Classic cart-pole system implemented by Rich Sutton et al.
Derived from http://incompleteideas.net/sutton/book/code/pole.c
permalink: https://perma.cc/C9ZM-652R
"""
__copyright__ = "Copyright 2020, Microsoft Corp."

## pyright: strict

import math
from gym.envs.classic_control import rendering
import random

import numpy as np

# Constants
GRAVITY = 9.8  # a classic...
CART_MASS = 0.31  # kg
POLE_MASS = 0.055  # kg
TOTAL_MASS = CART_MASS + POLE_MASS
POLE_HALF_LENGTH = 0.4 / 2  # half the pole's length in m
POLE_MASS_LENGTH = POLE_MASS * POLE_HALF_LENGTH
FORCE_MAG = 1.0
STEP_DURATION = 0.02  # seconds between state updates (20ms)
TRACK_WIDTH = 1.0  # m
FORCE_NOISE = 0.02  # % of FORCE_MAG

# Model parameters
class CartPoleModel():
    def __init__(
        self,
        initial_cart_position = 0,
        initial_pole_angle = np.pi,
        target_pole_position = 0,
    ):
        # cart position (m)
        self._cart_position = initial_cart_position

        # cart velocity (m/s)
        self._cart_velocity = 0

        # cart angle (rad)
        self._pole_angle = initial_pole_angle

        # pole angular velocity (rad/s)
        self._pole_angular_velocity = 0

        # pole position (m)
        self._pole_center_position = 0

        # pole velocity (m/s)
        self._pole_center_velocity = 0

        # target pole position (m)
        self._target_pole_position = target_pole_position

        self.state = [0,0,0,0]
        self.viewer = None

    def step(self, command: float):
        # We are expecting the input command to be -1 or 1,
        # but we'll support a continuous action space.
        # Add a small amount of random noise to the force so
        # the policy can't succeed by simply applying zero
        # force each time.
        force = FORCE_MAG * (command + random.uniform(-0.02, 0.02))

        cosTheta = math.cos(self._pole_angle)
        sinTheta = math.sin(self._pole_angle)

        temp = (
            force + POLE_MASS_LENGTH * self._pole_angular_velocity ** 2 * sinTheta
        ) / TOTAL_MASS
        angularAccel = (GRAVITY * sinTheta - cosTheta * temp) / (
            POLE_HALF_LENGTH * (4.0 / 3.0 - (POLE_MASS * cosTheta ** 2) / TOTAL_MASS)
        )
        linearAccel = temp - (POLE_MASS_LENGTH * angularAccel * cosTheta) / TOTAL_MASS

        self._cart_position = self._cart_position + STEP_DURATION * self._cart_velocity
        self._cart_velocity = self._cart_velocity + STEP_DURATION * linearAccel

        self._pole_angle = (
            self._pole_angle + STEP_DURATION * self._pole_angular_velocity
        )
        self._pole_angular_velocity = (
            self._pole_angular_velocity + STEP_DURATION * angularAccel
        )

        # Use the pole center, not the cart center, for tracking
        # pole center velocity.
        self._pole_center_position = (
            self._cart_position + math.sin(self._pole_angle) * POLE_HALF_LENGTH
        )
        self._pole_center_velocity = (
            self._cart_velocity
            + math.sin(self._pole_angular_velocity) * POLE_HALF_LENGTH
        )
        self.state = [self._cart_position, self._cart_velocity, self._pole_angle, self._pole_angular_velocity]
        self.render()
    
    def render(self, mode='human'):
        self.x_threshold = 10000
        self.world_width = self.x_threshold * 2
        self.screen_width = 600
        self.screen_height = 400
        self.scale = self.screen_width/self.world_width

        carty = 100  # TOP OF CART
        polewidth = 10.0
        polelen = 100
        cartwidth = 50.0
        cartheight = 30.0

        if self.viewer is None:

            self.viewer = rendering.Viewer(self.screen_width, self.screen_height)
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
            self.track = rendering.Line((0, carty), (self.screen_width, carty))
            self.track.set_color(0, 0, 0)
            self.viewer.add_geom(self.track)


            self.limit_right = rendering.Line((self.x_threshold * self.scale +self.screen_width / 4.0, -carty), 
                                            (self.x_threshold * self.scale+ self.screen_width / 4.0, 2*carty))
            self.viewer.add_geom(self.limit_right)
            self.limit_left = rendering.Line((self.screen_width / 4.0, -carty), 
                                            (self.screen_width / 4.0, 2*carty))
            self.viewer.add_geom(self.limit_left)

            self._pole_geom = pole

        if self.state is None:
            return None

        # Edit the pole polygon vertex
        pole = self._pole_geom
        l, r, t, b = -polewidth / 2, polewidth / 2, polelen - polewidth / 2, -polewidth / 2
        pole.v = [(l, b), (l, t), (r, t), (r, b)]

        x = self.state
        cartx = x[0] * self.scale + self.screen_width / 2.0  # MIDDLE OF CART
        self.carttrans.set_translation(cartx, carty)
        self.poletrans.set_rotation(-x[2])

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')
        
cartpole = CartPoleModel()


for _ in range(10000):
    cartpole.step(0)