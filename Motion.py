from pylab import *
from random import gauss
from utils import *

class UnicycleModel:

    def __init__(self, noise):
        self.noise = noise

    def simMove(self, pos, cmd):
        """
        Simulate the robot moving with noise
        """
        x = pos[0]
        y = pos[1]
        a = pos[2]

        forward = cmd[0]
        turn = cmd[1]

        forwardNoise = gauss(0, self.noise[0])
        turnNoise = gauss(0, self.noise[1])

        a = a + (turn + turnNoise)
        x = x + cos(a) * (forward + forwardNoise)
        y = y + sin(a) * (forward + forwardNoise)
        return array([x, y, a])

    def move(self, pos, cmd):
        """
        The motion model without noise, f(x,u)
        """
        x = pos[0]
        y = pos[1]
        a = pos[2]

        forward = cmd[0]
        turn = cmd[1]

        a = a + turn
        x = x + cos(a) * forward
        y = y + sin(a) * forward
        return array([x, y, a])

    def jacobianPosition(self, pos, cmd):
        """
        The jacobian with respect to position
        df(x,u)/dx
        """
        x = pos[0]
        y = pos[1]
        a = pos[2]
        f = cmd[0]
        t = cmd[1]
        F = np.array([[1, 0, f * sin(a)], [0, 1, f * cos(a)], [0, 0, 1]])
        return F

    def covariance(self, pos):
        a = pos[2]
        return np.diag([abs(self.noise[0] * cos(a)), abs(self.noise[0] * sin(a)), self.noise[1]])
