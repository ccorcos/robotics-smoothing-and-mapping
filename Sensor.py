from pylab import *
from random import gauss

# simulated laser range sensor


class LaserSensorSim:

    def __init__(self, options):

        self.type = options["type"]

        self.maxDistance = options["maxDistance"]
        self.maxAngle = options["maxAngle"]

        self.distanceNoise = options["distanceNoise"]
        self.angleNoise = options["angleNoise"]

    # simulate sensing
    def simSense(self, pos, landmarks):

        x = pos[0]
        y = pos[1]
        a = pos[2]
        xy = np.array([x, y])

        sensed = []

        for i in range(len(landmarks)):

            lm = landmarks[i]  # landmark i
            d = norm(lm - xy)  # distance to the landmark

            # angle to the landmark relative to a
            # t = arccos(dot(lm - xy, [cos(a), sin(a)]) / d)
            # t = arcsin(cross(lm - xy, [cos(a), sin(a)]) / d)

            c = np.array(lm - xy)
            b = np.array([cos(a), sin(a)])

            # sign(sintheta)*acos(costheta)
            t = -sign(cross(c, b) / d) * arccos(dot(c, b) / d)

            if d < self.maxDistance and abs(t) <= self.maxAngle:
                angleNoise = gauss(0, self.angleNoise)
                distanceNoise = gauss(0, self.distanceNoise)

                # senseXY = xy + \
                #     np.array([cos(a + t + angleNoise),
                #               sin(a + t + angleNoise)]) \
                #     * (d + distanceNoise)
                sensed.append(
                    {'which': i, "where": list([(d + distanceNoise), (t + angleNoise)])})

        return sensed
