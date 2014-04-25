from pylab import *

# landmarks = [
#       {
#           "type":"something",
#           "pos": [x,y]
#       }
#   ]

def randomLandmarks(n, scale, lmType):
    landmarks = [{"type": lmType,
                  "pos": array([random(), random()]) * scale} for n in range(n)]
    return landmarks


class Map:

    def __init__(self, scale, landmarkTypes):
        self.scale = scale
        Ltypes = []
        L = []
        for lmType in landmarkTypes:
            Ltypes.append(lmType['type'])
            L = L + randomLandmarks(lmType['n'], self.scale, lmType['type'])
        self.landmarks = L
        self.landmarkTypes = Ltypes

