from pylab import *
from random import gauss


class UnicycleModel:

    def __init__(self, options):
        self.noise = options['noise']

    def move(self, pos, cmd, noise=False):
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

    def jacobian(self, pos):
        pass
