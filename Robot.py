from pylab import *
from random import gauss


def unicycleModel(pos, command, noise=[0, 0]):
    x = pos[0]
    y = pos[1]
    a = pos[2]

    forward = command[0]
    turn = command[1]

    forwardNoise = gauss(0, noise[0])
    turnNoise = gauss(0, noise[1])

    a = a + (turn + turnNoise)
    x = x + cos(a) * (forward + forwardNoise)
    y = y + sin(a) * (forward + forwardNoise)
    return [x, y, a]


class RobotState:
    # keep track of which position and observations.

    def __init__(self, pos, obs):
        self.pos = pos
        self.obs = obs


class Robot:

    def __init__(self, options):

        # initial x, y, angle
        initialState = RobotState(options['initialPosition'], [])

        self.motionModel = options['motionModel']
        self.state = [initialState]
        self.motionNoise = options['motionNoise']

        self.sensors = options['sensors']

    # move the robot
    def move(self, command):
        pos = self.state[-1].pos
        newpos = self.motionModel(pos, command)
        newState = RobotState(newpos, [])
        self.state.append(newState)

    # tell the simulator where the robot is with noise
    def moveSim(self, pos, command):
        return self.motionModel(pos, command, self.motionNoise)

    # simulate sensing
    def simSense(self, simMap):
        pos = self.state[-1].pos
        obs = {}
        for sensor in self.sensors:
            obs[sensor.type] = sensor.simSense(
                pos, simMap.landmarks[sensor.type])
        self.state[-1].obs = obs

    def trajectory(self):
        traj = []
        for s in self.state:
            traj.append(s.pos)
        return np.array(traj)[:, 0:2]  # hack off the angle

    #
    #
    #
    #
    #
    #

    def realTrajectory(self):
        traj = []
        for state in self.realState:
            traj.append(state.pos)
        return np.array(traj)[:, 0:2]  # hack off the angle

    def drawReadTrajectory(self):
        traj = self.realTrajectory()
        plt.plot(traj[:, 0], traj[:, 1], color="green")

    def idealTrajectory(self):
        traj = []
        for state in self.idealState:
            traj.append(state.pos)
        return np.array(traj)[:, 0:2]  # hack off the angle

    def drawIdealTrajectory(self, ax):
        traj = self.idealTrajectory()
        ax.plot(traj[:, 0], traj[:, 1], color="green")

    def drawRealPosition(self):
        # draw a triangle oriented appropriately
        state = self.realState[-1]
        plt.scatter(state.pos[0], state.pos[1], s=50, marker="o", color="blue")
        a = state.pos[2]
        head = np.array([cos(a), sin(a)]) * 0.2
        start = np.array(state.pos[0:2])
        end = start + head
        pts = np.array([start, end])
        plt.plot(pts[:, 0], pts[:, 1], color="blue")

    def drawIdealPosition(self, ax):
        # draw a triangle oriented appropriately
        state = self.idealState[-1]
        ax.scatter(state.pos[0], state.pos[1], s=50, marker="o", color="red")
        a = state.pos[2]
        head = np.array([cos(a), sin(a)]) * 0.2
        start = np.array(state.pos[0:2])
        end = start + head
        pts = np.array([start, end])
        ax.plot(pts[:, 0], pts[:, 1], color="red")

    def drawIdealObs(self, fig):
        # draw a triangle oriented appropriately
        state = self.idealState[-1]
        observations = np.array(state.obs)[:, 0:2]  # scape off landmark index
        pos = np.array(state.pos)[0:2]  # scrape off angle

        # draw an ellipse oriented correctly with appropriate size
        for obs in observations:
            # perpendicular error depends on distance, draw 2 stdevs
            d = norm(pos - obs)
            angleErr = d * self.senseAngleNoise * 2
            distErr = self.senseDistanceNoise * 2

            # get angle
            ang = dot(pos - obs, [1, 0]) / d
            fig.gca().add_artist(
                Ellipse(xy=tuple(obs), width=distErr, height=angleErr, angle=ang, color="green", alpha=0.5))
