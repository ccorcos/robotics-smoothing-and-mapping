-from pylab import *
from random import gauss


class UnicycleModel:

    def __init__(self, options):
        self.noise = options['noise']

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
        return [x, y, a]

    def motionModel(self, pos, cmd):
        """
        The motion model without noise, f(x,u)
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
        return [x, y, a]

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

    def adjust(self, X, C):
        # adjust for maholanobis
        # inner will transpose C
        # fix this.
        return inner(sqrt(C), X)

    def noiseCovariance(self, pos):
        a = pos[2]
        return np.diag([abs(self.noise[0] * cos(a)), abs(self.noise[0] * sin(a)), self.noise[1]])
