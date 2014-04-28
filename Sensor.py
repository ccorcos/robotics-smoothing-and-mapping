from pylab import *
from random import gauss
from utils import *


class LaserSensorSim:

    def __init__(self, noise, sensorType, maxDistance, maxAngle):

        self.noise = noise  # distance, angle standard deviations
        self.sensorType = sensorType
        self.maxDistance = maxDistance
        self.maxAngle = maxAngle

    # simulate sensing
    def simSense(self, pos, simMap):
        """simulate sensing, given a map"""
        landmarks = simMap.landmarks

        x = pos[0]
        y = pos[1]
        a = pos[2]
        xy = np.array([x, y])

        sensed = []
        for i in range(len(landmarks)):
            landmark = landmarks[i]  # landmark i
            if landmark['type'] == self.sensorType:

                lm = landmark['pos']
                c = np.array(lm - xy)
                d = norm(c)  # distance to the landmark

                # t = arctan2(y,x)
                t = arctan2(c[1], c[0]) - a
                t = wrapAngle(t)

                #  check the sensor constraints
                if d < self.maxDistance and abs(t) <= self.maxAngle:
                    distanceNoise = gauss(0, self.noise[0])
                    angleNoise = gauss(0, self.noise[1])

                    obs = array([(d + distanceNoise), (t + angleNoise)])
                    descriptor = i  # the index to the map

                    sensed.append(
                        {"obs": obs, "descriptor": descriptor, "sensorType": self.sensorType})

        return sensed

    def sense(self, pos, landmark):
        """The observation model for this sensor, h(x,l)"""

        x = pos[0]
        y = pos[1]
        a = pos[2]
        xy = np.array([x, y])
        c = np.array(landmark - xy)
        d = norm(c)  # distance to the landmark

        # b = np.array([cos(a), sin(a)])

        # sign(sintheta)*acos(costheta)
        # t = -sign(cross(c, b) / d) * arccos(dot(c, b) / d)

        # t = arctan2(y,x)
        t = arctan2(c[1], c[0]) - a
        t = wrapAngle(t)

        return array([d, t])

    def deadReckon(self, pos, obs):
        """Dead reckon the landmark position from the position and observation"""
        xy = np.array(pos[0:2])
        a = np.array(pos[2])
        d = obs[0]
        t = obs[1]
        lm = np.array([d * cos(a + t), d * sin(a + t)]) + xy
        return lm

    def jacobianPosition(self, pos, lm):
        """The jacobian with respect to position, dh(x,l)/dx"""
        x = pos[0]
        y = pos[1]
        a = pos[2]
        w = lm[0]  # lmx
        z = lm[1]  # lmy

        return array([[(-w + x)/sqrt((w - x)**2 + (-y + z)**2), (y - z)/sqrt((w - x)**2 + (-y + z)**2), 0], [-(y - z)/((w - x)**2 + (-y + z)**2), -(w - x)/((w - x)**2 + (-y + z)**2), -1]])

    def jacobianLandmark(self, pos, lm):
        """The jacobian with respect to the landmark, dh(x,l)/dl"""
        # FIX

        x = pos[0]
        y = pos[1]
        a = pos[2]
        w = lm[0]  # lmx
        z = lm[1]  # lmy

        return array([[(w - x)/sqrt((w - x)**2 + (-y + z)**2), (-y + z)/sqrt((w - x)**2 + (-y + z)**2)], [(y - z)/((w - x)**2 + (-y + z)**2), (w - x)/((w - x)**2 + (-y + z)**2)]])

    def covariance(self):
        return array([[self.noise[0] ** 2, 0], [0, self.noise[1] ** 2]])
