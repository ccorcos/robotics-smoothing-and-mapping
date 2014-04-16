from pylab import *
from random import gauss
from scipy.optimize import *
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
        self.landmarks = []
        self.plotLandmarks = {}
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
            cost = []
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
                    cost.append(thisCost)

                for observation in thisState['obs']:
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

                    cost.append(thisCost)

                # for lm in self.landmarks:
                #     observation = findKV(
                #         thisState["obs"], "map index", lm["map index"])
                #     if observation:
                #         whatISensed = observation["sensor pos"]
                #         whereIThinkTheLandmarkIs = findKV(self.landmarks,
                #                                           "map index",
                #                                           observation["map index"])["pos"]
                #         theSensorUsed = self.sensorOfType(observation["type"])

                #         whatIShouldHaveSensed = theSensorUsed.obsIdeal(
                #             whereIThinkIAm,
                #             whereIThinkTheLandmarkIs)

                #         thisCost = mahalanobis(
                #             whatIShouldHaveSensed,
                #             whatISensed,
                #             theSensorUsed.noiseCovariance(whatIShouldHaveSensed))
                #         j = findKVindex(self.landmarks,
                #                         'map index',
                #                         observation["map index"])
                # print j
                #         cost.append(thisCost)
                #     else:
                #         cost.append(0)

            print 'cost: ', sum(cost)
            # return sum(cost)
            return cost

        # while True:
        #     c = f(X0)
        #     print c
        #     n = len(X0)
        #     for i in range(n):
        #         p = [0] * n
        #         p[i] = 0.01
        #         p = np.array(p)
        #         if f(X0 + p) < c:
        #             X0 = X0 + p
        #         elif f(X0 - p) < c:
        #             X0 = X0 - p

        Xf = leastsq(f, X0)

        # update landmarks for plotting
        self.plotLandmarks = {}
        for l in self.landmarks:
            if l["type"] in self.plotLandmarks:
                self.plotLandmarks[l["type"]].append(l["pos"])
            else:
                self.plotLandmarks[l["type"]] = [l["pos"]]

    def linearSAM(self):

        N = len(self.state)
        M = len(self.landmarks)
        # A = {'pos': [], 'obs': []}
        A = []
        b = []

        for i in range(1, N):
            whereIAm = self.state[i]
            whereIWas = self.state[i - 1]
            whereIShouldBe = self.motionModel.move(whereIWas['pos'],
                                                   whereIWas['cmd'])
            ai = whereIAm['pos'] - whereIShouldBe

            Fi = self.motionModel.jacobianPos(whereIWas['pos'],
                                              whereIWas['cmd'])
            Gi = eye(f.shape)

            Ci = self.motionModel.noiseCovariance(whereIWas['pos'])
            Fi = self.motionModel.adjust(F, C)
            Gi = self.motionModel.adjust(G, C)
            ai = self.motionModel.adjust(ai, C)

            Ai = [None] * (M + N - 1)
            Ai[i] = Gi
            Ai[i - 1] = Fi
            A.append(Ai)
            b.append(ai)
            # A['pos'].append([i, Fi, Gi, ai])

            for j in range(len(whereIAm['obs'])):
                observation in whereIAm['obs'][j]:
                whatIObserved = observation['sensor pos']
                whereIThinkTheLandmarkIs = findKV(self.landmarks,
                                                  "map index",
                                                  observation["map index"])["pos"]
                theSensorUsed = self.sensorOfType(observation["type"])

                whatIShouldObserve = theSensorUsed.obsIdeal(whereIAm['pos'],
                                                            whereIThinkTheLandmarkIs)
                ci = whatIObserved - whatIShouldObserve

                Hi = theSensorUsed.jacobianPos(whereIAm['pos'], whatIObserved)
                Ji = theSensorUsed.jacobianObs(whereIAm['pos'], whatIObserved)

                Ci = theSensorUsed.noiseCovariance(whereIWas['pos'])
                Hi = theSensorUsed.adjust(Hi, Ci)
                Ji = theSensorUsed.adjust(Ji, Ci)
                ci = theSensorUsed.adjust(ci, Ci)
                lm = observation["map index"]

                Ai = [None] * (M + N - 1)
                Ai[i] = Hi
                Ai[lm + N - 1] = Ji
                A.append(Ai)
                b.append(bi)

        # now we need to flatten it out a little bit
        A2 = []
        b2 = []
        for i in range(M + N - 1):
            bi = b[i]
            Ai = A[i]
            n = len(bi)
            for j in range(n):
                b2.append(bi[j])
            A2i = zeros((n, M + N - 1))
            for j in range(len(Ai)):
                if Ai[j] != None:
                    for r in range(n):
                        for c in range(n):
                            A2i[r][n * j + c] = Ai[j][r][c]
            A2 = A2 + A2i.tolist()

        delta = inner(pinv(np.array(A2)).T, np.array(b))

        # FUCK!!!!! How do I update this now?!

        # update landmarks for plotting
        self.plotLandmarks = {}
        for l in self.landmarks:
            if l["type"] in self.plotLandmarks:
                self.plotLandmarks[l["type"]].append(l["pos"])
            else:
                self.plotLandmarks[l["type"]] = [l["pos"]]


def mahalanobis(x, y, C):
    d = np.array(x) - np.array(y)
    D = pinv(np.array(C))
    m = inner(inner(d, D), d)
    a = sqrt(m)
    # if isnan(a):
    #     print x
    #     print y
    #     print C
    #     print d
    #     print D
    #     print m
    #     wait()
    return a

    return
