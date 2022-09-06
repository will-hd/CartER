from time import sleep
from gym.envs.classic_control import rendering
import math
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

masspole = 100 #kg
length = 1 #m
gravity = 9.8 #ms^-2

class Cart:
    """
    Compute everything in SI units then scale to steps.
    """
    def __init__(self) -> None:
        # self.masspole = 1
        # self.length = 1
        # self.gravity = 9.8
        self.dt = 0.1
        self.state = [0, 0, np.pi, 0.0]
        self.time = 0

        self.viewer = None
        self.RAIL_STEP_LENGTH = 10000 # steps, lets assume for now this is 1m
        self.RAIL_M_LENGTH = 1 # metres
        self.STEP_LENGTH = self.RAIL_M_LENGTH/self.RAIL_STEP_LENGTH # metres/step 
        self.WORLD_WIDTH = self.RAIL_M_LENGTH *5# self.STEP_LENGTH *2 # i.e. currently 2m

    @staticmethod
    def pole_dynamics(y, t, x_ddot, masspole, length, gravity):
        """
        Equations of motion for cart
        """
        theta, theta_dot = y

        # For the interested reader:
        # https://coneural.org/florian/papers/05_cart_pole.pdf
        # Eq (14):*******minus sign
        # Theta zero at top and +ve going clockwise from vertical
        thetaacc = 3.0 / (4.0 * masspole * length**2) * (
            masspole*length * (gravity*math.sin(theta) - x_ddot*math.cos(theta)) 
        )
        dydt = [theta_dot, thetaacc]
        return dydt

    def cart_dynamics(self, xacc) -> tuple:
        x_dot = self.state[1] + self.dt * xacc
        x = self.state[0] + x_dot * self.dt

        return (x, x_dot)

    def step(self, xacc):
        self.time += self.dt
        x, x_dot = self.cart_dynamics(xacc)
        time_arr = np.linspace(self.time-self.dt, self.time, 100001)
        sol = odeint(self.pole_dynamics, self.state[2:], time_arr, args=(xacc,masspole,length,gravity))

        self.state = (x, x_dot, sol[-1, 0], sol[-1, 1])
        print(f"Time: {self.time}")
        print(self.state)
        # sleep(0.05)
        self.render()

    def render(self, mode='human'):

        self.screen_width = 600
        self.screen_height = 600

        self.SCALE = self.screen_width/self.WORLD_WIDTH # pixels/2 metre

        carty = self.screen_height/2  # TOP OF CART
        polewidth = 0.02*self.SCALE
        polelen = length*self.SCALE
        cartwidth = 0.1*self.SCALE
        cartheight = 0.06*self.SCALE

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


            self.limit_right = rendering.Line((self.RAIL_M_LENGTH * self.SCALE/2 + self.screen_width /2, carty/2), 
                                              (self.RAIL_M_LENGTH * self.SCALE/2 + self.screen_width /2, carty+carty/2))
            self.viewer.add_geom(self.limit_right)
            self.limit_left = rendering.Line((-self.RAIL_M_LENGTH * self.SCALE/2 + self.screen_width /2, carty/2), 
                                             (-self.RAIL_M_LENGTH * self.SCALE/2 + self.screen_width /2, carty+carty/2))
            self.viewer.add_geom(self.limit_left)

            self._pole_geom = pole

        if self.state is None:
            return None

        # Edit the pole polygon vertex
        pole = self._pole_geom
        l, r, t, b = -polewidth / 2, polewidth / 2, polelen - polewidth / 2, -polewidth / 2
        pole.v = [(l, b), (l, t), (r, t), (r, b)]

        x = self.state
        cartx = x[0] * self.SCALE + self.screen_width / 2.0  # MIDDLE OF CART
        self.carttrans.set_translation(cartx, carty)
        self.poletrans.set_rotation(-x[2])

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

test_cart = Cart()

# for _ in range(10):
#     test_cart.step(1)

# for _ in range(25):
#     test_cart.step(-1)

# for _ in range(40):
#     test_cart.step(1)

# for _ in range(20):
#     test_cart.step(-1)
for _ in range(1000):
    test_cart.step(0.01)