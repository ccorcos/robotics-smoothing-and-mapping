from pylab import *
from random import gauss
from utils import *

class LaserSensorSim:

    def __init__(self, noise, sensorType, maxDistance, maxAngle):

        self.noise = noise # distance, angle standard deviations
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
                # distance to the landmark
                d = norm(lm - xy)

                # compute the angle to the landmark
                c = np.array(lm - xy)
                b = np.array([cos(a), sin(a)])

                # sign(sintheta)*acos(costheta)
                t = -sign(cross(c, b) / d) * arccos(dot(c, b) / d)

                #  check the sensor constraints
                if d < self.maxDistance and abs(t) <= self.maxAngle:
                    distanceNoise = gauss(0, self.noise[0])
                    angleNoise = gauss(0, self.noise[1])

                    obs = array([(d + distanceNoise), (t + angleNoise)])
                    descriptor = i # the index to the map

                    sensed.append({"obs":obs, "descriptor":descriptor, "sensorType":self.sensorType})

        return sensed

    def sense(self, pos, landmark):
        """The observation model for this sensor, h(x,l)"""

        x = pos[0]
        y = pos[1]
        a = pos[2]
        xy = np.array([x, y])

        d = norm(landmark - xy)  # distance to the landmark

        c = np.array(landmark - xy)
        b = np.array([cos(a), sin(a)])

        # sign(sintheta)*acos(costheta)
        t = -sign(cross(c, b) / d) * arccos(dot(c, b) / d)

        distanceNoise = gauss(0, self.noise[0])
        angleNoise = gauss(0, self.noise[1])
        return array([d, t])

    def deadReckon(self,pos,obs):
        """Dead reckon the landmark position from the position and observation"""
        xy = np.array(pos[0:2])
        a = np.array(pos[2])
        d = obs[0]
        t = obs[1]
        lm = np.array([d * cos(a + t), d * sin(a + t)]) + xy
        return lm

    def jacobianPosition(self, pos, lm):
        """The jacobian with respect to position, dh(x,l)/dx"""
        # FIX
        return np.array([[1, 0], [0, 1]])
    

    def jacobianLandmark(self, pos, lm):
        """The jacobian with respect to the landmark, dh(x,l)/dl"""
        # FIX
        return np.array([[1, 0], [0, 1]])

    def covariance(self):
        return array([[self.noise[0]**2, 0], [0, self.noise[1]**2]])
