
from pylab import *
from random import gauss
from utils import *


def randomLandmarks(n, scale, lmType):
    landmarks = [{"type": lmType,
                  "pos": np.array([random(), random()]) * scale} for n in range(n)]
    return landmarks


# map
#   landmarks = [
#       {
#           "type":"",
#           "pos": [x,y]
#       }
#   ]

class Map:

    def __init__(self, options):
        self.scale = options['scale']
        Ltypes = []
        L = []
        for lmType in options['landmarkTypes']:
            Ltypes.append(lmType['type'])
            L = L + randomLandmarks(lmType['n'], self.scale, lmType['type'])
        self.landmarks = L
        self.landmarkTypes = Ltypes

        lm4Plot = {}
        for lm in L:
            if lm['type'] in lm4Plot:
                lm4Plot[lm['type']].append(lm["pos"])
            else:
                lm4Plot[lm['type']] = [lm["pos"]]

        self.plotLandmarks = lm4Plot


# map = [{type: pos:}]
