"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from http://incompleteideas.net/sutton/book/code/pole.c
permalink: https://perma.cc/C9ZM-652R
"""
import time
import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
import platform
from typing import Any, cast

if platform.system() == "Windows":
    from gym.envs.classic_control import rendering
else:
    from pyglet.canvas.xlib import NoSuchDisplayException

    # Display detection
    try:
        from gym.envs.classic_control import rendering
    except NoSuchDisplayException:
        rendering = cast(Any, None)

class CartPoleEnv(gym.Env):
    """
    Description:
        A pole is attached by an un-actuated joint to a cart, which moves along
        a frictionless track. The pendulum starts upright, and the goal is to
        prevent it from falling over by increasing and reducing the cart's
        velocity.

    Source:
        This environment corresponds to the version of the cart-pole problem
        described by Barto, Sutton, and Anderson

    Observation:
        Type: Box(4)
        Num     Observation               Min                     Max
        0       Cart Position             -4.8                    4.8
        1       Cart Velocity             -Inf                    Inf
        2       Pole Angle                -0.418 rad (-24 deg)    0.418 rad (24 deg)
        3       Pole Angular Velocity     -Inf                    Inf
    Actions:
        Type: Discrete(2)
        Num   Action
        0     Push cart to the left
        1     Push cart to the right

        Note: The amount the velocity that is reduced or increased is not
        fixed; it depends on the angle the pole is pointing. This is because
        the center of gravity of the pole increases the amount of energy needed
        to move the cart underneath it

    Reward:
        Reward is 1 for every step taken, including the termination step

    Starting State:
        All observations are assigned a uniform random value in [-0.05..0.05]

    Episode Termination:
        Pole Angle is more than 12 degrees.
        Cart Position is more than 2.4 (center of the cart reaches the edge of
        the display).
        Episode length is greater than 200.
        Solved Requirements:
        Considered solved when the average return is greater than or equal to
        195.0 over 100 consecutive trials.
    """

    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}

    def __init__(self, unrestricted_mode=False, skip_steps=None, end_reward=1.0, x_threshold=15000, weak=False):
        
        self.gravity = 9.8
        self.masscart = 1.0
        self.masspole = 0.1
        
        self.unrestricted_mode = unrestricted_mode
        self.skip_steps = skip_steps
        #self.skip_steps = lambda: np.abs(np.random.normal(loc=4.0, scale=1.5, size=size)).astype('int')
        self.end_reward = end_reward
        self.weak = weak
        
        self.total_mass = self.masspole + self.masscart
        self.length = 0.5  # actually half the pole's length
        self.polemass_length = self.masspole * self.length
        
        self.velocity_step = 1

        self.tau = 0.2  # seconds between state updates
        self.kinematics_integrator = "euler"

        self.x_threshold = x_threshold
        # Set max steps as failure mode:
        self.max_steps = 4000
        self.step_count = 0
        # Angle limit set to 2 * theta_threshold_radians so failing observation
        # is still within bounds.
        high = np.array(
            [
                201,
                np.finfo(np.float32).max
            ],
            dtype=np.float32,
        )
        low = np.array(
            [
                0,
                -np.finfo(np.float32).max
            ],
            dtype=np.float32,
        )

        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self.seed()
        self.viewer = None
        self.state = None

        self.policy_update_steps = 0
        self.steps_beyond_done = None

        self.bins = [i*100 for i in range(201)]

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    

    def step(self, action):
        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        self.policy_update_steps += 1
        if self.policy_update_steps % 2048== 0:
            print("policy update")
        sim_steps = 1
        
        if self.skip_steps is not None:
            sim_steps = self.skip_steps()
            
        reward = 0.0
        done = False
            
    
        self.step_count +=1
        self.x, self.x_dot = self.state

        x_dot_step = self.velocity_step if action == 1 else -self.velocity_step if action == 0 else 0.0
        time.sleep(0.001)
        self.x_dot += x_dot_step
        self.x += self.tau * self.x_dot

    
        x_binned = np.digitize(self.x, self.bins) -1
        
        
        self.state = (self.x, self.x_dot)
        self.binned_state = (x_binned, self.x_dot)

        done = bool(
            self.x < 0
            or self.x > self.x_threshold
            or self.step_count > self.max_steps
            or self.policy_update_steps % 2048 == 0
        )

        # if done:
        #     print("Done")
        if not done:
            reward += self.calcreward()
        
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = self.end_reward
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward += 0.0
            
        sim_steps -= 1
        
        if self.weak:
            action = -1
                

        return np.array(self.binned_state, dtype=np.float32), reward, done, {}

    def calcreward(self):
        position = self.x
       
        rew = np.sin(position * np.pi / (2 * self.x_threshold))**12
        return rew


    def reset(self):
        self.state = (self.np_random.normal(loc=10000, scale=1000), self.np_random.uniform(low=-20, high=20))
        self.steps_beyond_done = None
        self.step_count = 0
        return np.array(self.state, dtype=np.float32)

    def render(self, mode="human"):
        screen_width = 600
        screen_height = 400
        world_width = self.x_threshold * 2
        
        # screen_width = int((sc/(20000*2))*world_width+400)
        
        scale = screen_width / world_width
        offset = self.x_threshold / 2 * scale
        carty = 100  # TOP OF CART
        polewidth = 10.0
        polelen = scale * (2 * self.length)
        cartwidth = 50.
        cartheight = 30.0

        if self.viewer is None:
            self.viewer = rendering.Viewer(screen_width, screen_height)
            # Cart
            l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
            axleoffset = cartheight / 4.0
            cart = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            self.carttrans = rendering.Transform()
            cart.add_attr(self.carttrans)
            self.viewer.add_geom(cart)
            
            # Pole
            l, r, t, b = (
                -polewidth / 2,
                polewidth / 2,
                polelen - polewidth / 2,
                -polewidth / 2,
            )
            pole = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            pole.set_color(0.8, 0.6, 0.4)
            self.poletrans = rendering.Transform(translation=(0, axleoffset))
            pole.add_attr(self.poletrans)
            pole.add_attr(self.carttrans)
            self.viewer.add_geom(pole)
            self.axle = rendering.make_circle(polewidth / 2)
            self.axle.add_attr(self.poletrans)
            self.axle.add_attr(self.carttrans)
            self.axle.set_color(0.5, 0.5, 0.8)
            self.viewer.add_geom(self.axle)
            self.track = rendering.Line((0, carty), (screen_width, carty))
            self.track.set_color(0, 0, 0)
            self.viewer.add_geom(self.track)

            self.limit_right = rendering.Line((self.x_threshold * scale +offset, -carty), 
                                            (self.x_threshold * scale+ offset, 2*carty))
            self.viewer.add_geom(self.limit_right)
            self.limit_left = rendering.Line((offset, -carty), 
                                            (offset, 2*carty))
            self.viewer.add_geom(self.limit_left)

            self._pole_geom = pole

        if self.state is None:
            return None

        # Edit the pole polygon vertex
        pole = self._pole_geom
        l, r, t, b = (
            -polewidth / 2,
            polewidth / 2,
            polelen - polewidth / 2,
            -polewidth / 2,
        )
        pole.v = [(l, b), (l, t), (r, t), (r, b)]

        x = self.state
        cartx = x[0] * scale + offset # + screen_width / 2.0  # MIDDLE OF CART
        self.carttrans.set_translation(cartx, carty)
        # self.poletrans.set_rotation(-x[2])
        self.poletrans.set_rotation(0)

        return self.viewer.render(return_rgb_array=mode == "rgb_array")

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None