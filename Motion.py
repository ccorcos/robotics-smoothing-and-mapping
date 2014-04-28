from pylab import *
from random import gauss
from utils import *

class UnicycleModel:

    def __init__(self, noise):
        self.noise = noise # [x, y, a] standard deviations

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
        a = wrapAngle(a)
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
        a = wrapAngle(a)
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
        return array([[1, 0, -f*sin(a)], [0, 1, f*cos(a)], [0, 0, 1]])

    def covariance(self, pos, cmd):
        # FIX: include pos in this covariance
        f = cmd[0]
        t = cmd[1]
        a = pos[2]

        forwardVariance = self.noise[0]**2
        turnVariance = self.noise[1]**2
        xVariance = abs(forwardVariance*cos(a+t)) + abs(f*turnVariance*sin(a+t))
        yVariance = abs(forwardVariance*sin(a+t)) + abs(f*turnVariance*cos(a+t))
        aVariance = turnVariance
        return diag([xVariance, yVariance, aVariance])
        # return np.diag([abs(self.noise[0]**2 * cos(a)), abs(self.noise[0]**2 * sin(a)), self.noise[1]**2])
        # return np.diag([self.noise[0]**2, self.noise[0]**2, self.noise[1]**2])

