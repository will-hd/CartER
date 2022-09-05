from gym.envs.classic_control import rendering
import math
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

masspole = 100 #kg
length = 1 #m
gravity = 9.8 #ms^-2

def cart_dydt(y, t, xacc, masspole, length, gravity):
    xacc = xacc
    theta, theta_dot = y

    # For the interested reader:
    # https://coneural.org/florian/papers/05_cart_pole.pdf
    # Eq (14):*******minus sign
    # Theta zero at top and +ve going clockwise from vertical
    thetaacc = 3.0 / (4.0 * masspole * length**2) * (
        masspole*length * (gravity*math.sin(theta) - xacc*math.cos(theta)) 
    )
    dydt = [theta_dot, thetaacc]

    # theta, omega = y

    # dydt = [omega, -0.5*np.sin(theta)]
    return dydt

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

        self.RAIL_LENGTH = 200 # steps, lets assume for now this is 1m
        self.STEP_LENGTH = 1/10000 # metres/step (one step in metric metres) 

        self.world_width = self.RAIL_LENGTH *2# self.STEP_LENGTH *2 # i.e. currently 2m


        self.viewer = None
        self.screen_width = 600
        self.screen_height = 400
        self.scale = self.screen_width/self.world_width # pixels/metre

    def step(self, xacc):
        self.time += self.dt
        x_dot = self.state[1] + self.dt * xacc
        x = self.state[0] + x_dot * self.dt

        time_arr = np.linspace(self.time-self.dt, self.time, 100001)
        sol = odeint(cart_dydt, self.state[2:], time_arr, args=(xacc,masspole,length,gravity))

        self.state = (x, x_dot, sol[-1, 0], sol[-1, 1])
        print(f"Time: {self.time}")
        # plt.plot(time_arr, sol[:, 0], 'b', label='theta(t)')
        self.render()
        # plt.show()

    def render(self, mode='human'):

        carty = 100  # TOP OF CART
        polewidth = 10.0
        polelen = length*self.scale
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


            self.limit_right = rendering.Line((self.RAIL_LENGTH * self.scale/2 +self.screen_width / 2, -carty), 
                                            (self.RAIL_LENGTH * self.scale+ self.screen_width / 4.0, 2*carty))
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

test_cart = Cart()

for _ in range(300):
    test_cart.step(1)
for _ in range(600):
    test_cart.step(-1)