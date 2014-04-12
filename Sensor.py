from pylab import *
from random import gauss

# simulated laser range sensor

# obs = [
#   {
#       'map index': 1,
#       'sensor pos': [d,t],
#       'type': "1"
#   }
# ]


class LaserSensorSim:

    def __init__(self, options):

        self.type = options["type"]

        self.maxDistance = options["maxDistance"]
        self.maxAngle = options["maxAngle"]

        self.noise = options['noise']  # distance, angle

    # simulate sensing
    def simSense(self, pos, simMap):
        landmarks = simMap.landmarks

        sensed = []
        for i in range(len(landmarks)):
            landmark = landmarks[i]  # landmark i
            if landmark['type'] == self.type:
                o = self.obs(pos, landmark['pos'], noise=True)
                if o:
                    sensed.append({'map index': i,
                                   'sensor pos': o,
                                   'type': self.type})

        return sensed

    def obs(self, pos, landmark, noise=False):

        x = pos[0]
        y = pos[1]
        a = pos[2]
        xy = np.array([x, y])

        d = norm(landmark - xy)  # distance to the landmark

        c = np.array(landmark - xy)
        b = np.array([cos(a), sin(a)])

        # sign(sintheta)*acos(costheta)
        t = -sign(cross(c, b) / d) * arccos(dot(c, b) / d)

        if d < self.maxDistance and abs(t) <= self.maxAngle:
            if noise:
                distanceNoise = gauss(0, self.noise[0])
                angleNoise = gauss(0, self.noise[1])
                return [(d + distanceNoise), (t + angleNoise)]
            else:
                return [d, t]
        else:
            return None

    def obsIdeal(self, pos, landmark):

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
        return [(d + distanceNoise), (t + angleNoise)]

    def jacobian():
        pass

    def noiseCovariance(self, obs):
        return np.diag(self.noise)
