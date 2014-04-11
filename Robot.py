from pylab import *
from random import gauss


class RobotState:
    # keep track of which position and observations.

    def __init__(self, pos, obs):
        self.pos = pos
        self.obs = obs

    def command(self, cmd):
        self.cmd = cmd


class Robot:

    def __init__(self, options):

        # initial x, y, angle
        initialState = RobotState(options['initialPosition'], [])

        self.sensors = options['sensors']
        self.motionModel = options['motionModel']
        self.state = [initialState]

    # move the robot
    def move(self, cmd):
        self.state[-1].command(cmd)  # keep track of the commands
        pos = self.state[-1].pos

        newpos = self.motionModel.move(pos, cmd)
        newState = RobotState(newpos, [])
        self.state.append(newState)

    # tell the simulator where the robot is with noise
    def simMove(self, pos, cmd):
        # move the robot as per usual, but tell the simulator where the robot
        # really is due to noise accumulation, etc.
        self.move(cmd)
        return self.motionModel.move(pos, cmd, noise=True)

    # simulate sensing. based on the real position.
    def simSense(self, simMap, pos):
        obs = {}
        for sensor in self.sensors:
            obs[sensor.type] = sensor.simSense(pos,
                                               simMap.landmarks[sensor.type])
        self.state[-1].obs = obs

    def trajectory(self):
        traj = []
        for s in self.state:
            traj.append(s.pos)
        return np.array(traj)[:, 0:2]  # hack off the angle

    def reset(self):
        initState = self.state[0]
        initState.obs = []
        initState.cmd = None
        self.state = [initState]

    def sensorOfType(self, s):
        for sensor in self.sensors:
            if sensor.type == s:
                return sensor
