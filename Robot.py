from pylab import *
from random import gauss
from scipy.optimize import leastsq
from utils import *


# state = {pos:[x,y,a], obs:[{'type':"1",'map index':4, 'sensor pos':[d,t]}], "cmd":[d,t]}


# state = [
#   {
#       "pos": [x,y,a],
#       "obs": [
#           {
#               'type':"1",
#               'map index': 4,
#               'sensor pos': [d,t]
#           }
#       ],
#       'cmd': [d,t]
#   }
# ]
# landmarks = [
#   {
#       'map index': 4,
#       'pos': [x,y],
#       'type': "1"
#   }
# ]
class Robot:

    def __init__(self, options):

        # initial x, y, angle
        initialState = {"pos": options['initialPosition'],
                        'obs': [],
                        'cmd': []}

        self.sensors = options['sensors']
        self.motionModel = options['motionModel']
        self.state = [initialState]
        self.landmarks = []
        self.plotLandmarks = {}

    # move the robot
    def move(self, cmd):
        self.state[-1]["cmd"] = cmd  # keep track of the commands
        pos = self.state[-1]["pos"]

        newpos = self.motionModel.move(pos, cmd)
        newState = {"pos": newpos}
        self.state.append(newState)

    # tell the simulator where the robot is with noise
    def simMove(self, pos, cmd):
        # move the robot as per usual, but tell the simulator where the robot
        # really is due to noise accumulation, etc.
        self.move(cmd)
        return self.motionModel.move(pos, cmd, noise=True)

    # simulate sensing. based on the real position.
    def simSense(self, simMap, pos):

        for sensor in self.sensors:
            sensorObs = sensor.simSense(pos, simMap)
            if "obs" not in self.state[-1]:
                self.state[-1]["obs"] = []
            self.state[-1]["obs"] = self.state[-1]["obs"] + sensorObs
            # returns [{"map index":, "sensor pos":, "type"}]

            for o in sensorObs:
                lm = findKV(self.landmarks, "map index", o["map index"])
                if not lm:
                    pos = self.state[-1]["pos"]
                    xy = np.array(pos[0:2])
                    a = np.array(pos[2])
                    d = o["sensor pos"][0]
                    t = o["sensor pos"][1]
                    posLm = np.array([d * cos(a + t), d * sin(a + t)]) + xy
                    self.landmarks.append({"map index": o["map index"],
                                           "pos": posLm,
                                           "type": o["type"]})
                    if o["type"] in self.plotLandmarks:
                        self.plotLandmarks[o["type"]].append(posLm)
                    else:
                        self.plotLandmarks[o["type"]] = [posLm]

    def trajectoryXY(self):
        traj = pluck(self.state, "pos")
        return np.array(traj)[:, 0:2]  # hack off the angle

    def trajectory(self):
        return np.array(pluck(self.state, "pos"))

    def reset(self):
        initState = self.state[0]
        initState["obs"] = []
        initState["cmd"] = None
        self.state = [initState]

    def sensorOfType(self, s):
        for sensor in self.sensors:
            if sensor.type == s:
                return sensor

    def wrap(self):
        positions = np.array(pluck(self.state, "pos"))
        landmarks = np.array(pluck(self.landmarks, "pos"))

        posShape = positions.shape
        posLen = positions.size
        lmShape = landmarks.shape
        lmLen = landmarks.size

        X = positions.reshape((1, posLen))[0].tolist() + \
            landmarks.reshape((1, lmLen))[0].tolist()
        return X, posShape, posLen, lmShape, lmLen

    def unwrap(self, X, posShape, posLen, lmShape, lmLen):
        poss = np.array(X[0:posLen]).reshape(posShape)
        lms = np.array(X[posLen:]).reshape(lmShape)

        for i in range(len(poss)):
            self.state[i]["pos"] = poss[i]
        for i in range(len(lms)):
            self.landmarks[i]["pos"] = lms[i]

    def nonlinearSAM(self):

        X0, posShape, posLen, lmShape, lmLen = self.wrap()

        def f(X):
            self.unwrap(X, posShape, posLen, lmShape, lmLen)

            N = len(self.state)
            cost = [0] * len(X)
            for i in range(N):
                thisState = self.state[i]
                whereIThinkIAm = thisState['pos']
                if i > 0:
                    lastState = self.state[i - 1]
                    whereIThoughtIWas = lastState["pos"]
                    lastCommand = lastState["cmd"]
                    whereIShouldBe = self.motionModel.move(
                        whereIThoughtIWas, lastCommand)
                    thisCost = mahalanobis(
                        whereIShouldBe,
                        whereIThinkIAm,
                        self.motionModel.noiseCovariance(whereIThoughtIWas))
                    cost[i] = cost[i] + thisCost

                for observation in thisState["obs"]:
                    whatISensed = observation["sensor pos"]
                    whereIThinkTheLandmarkIs = findKV(self.landmarks,
                                                      "map index",
                                                      observation["map index"])["pos"]
                    theSensorUsed = self.sensorOfType(observation["type"])

                    whatIShouldHaveSensed = theSensorUsed.obsIdeal(
                        whereIThinkIAm,
                        whereIThinkTheLandmarkIs)

                    thisCost = mahalanobis(
                        whatIShouldHaveSensed,
                        whatISensed,
                        theSensorUsed.noiseCovariance(whatIShouldHaveSensed))
                    j = findKVindex(self.landmarks,
                                    'map index',
                                    observation["map index"])
                    cost[N + j] = cost[N + j] + thisCost
            return cost

        Xf = leastsq(f, X0)


def mahalanobis(x, y, C):
    # print x, y, C
    # input("sdf")

    # try:
    d = np.array(x) - np.array(y)
    D = pinv(np.array(C))
    m = inner(inner(d, D), d)
    a = sqrt(m)
    return a
    # except:
    #     print x, y, C
    # print d, D
    #     input("sdf")

    return
