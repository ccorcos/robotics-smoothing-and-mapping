
from pylab import *
from random import gauss


def randomLandmarks(n, scale):
    landmarks = []
    [landmarks.append([random(), random()]) for n in range(n)]
    landmarks = np.array(landmarks)
    landmarks = landmarks * scale
    return landmarks


class Map:

    def __init__(self, options):
        self.scale = options['scale']
        Ltypes = []
        L = {}
        for lmType in options['landmarkTypes']:
            Ltypes.append(lmType['type'])
            L[lmType['type']] = randomLandmarks(lmType['n'], self.scale)
        self.landmarks = L
        self.landmarkTypes = Ltypes
