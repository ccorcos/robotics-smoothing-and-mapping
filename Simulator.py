from pylab import *
import os
import pickle

from Plot import *


class Simulator:

    def __init__(self, options):
        self.map = options['map']
        self.robots = options['robots']
        self.plot = Plot()
        self.terminal = options['terminal']
        self.state = [[r.state[0]] for r in self.robots]

    def recordTrajectory(self):
        commands = []  # forward and turn

        robot = self.robots[0]
        # pos = self.state[0][0].pos

        self.plot.start("Record Trajectory")
        self.plot.drawMap(self.map)
        self.plot.drawRobotTrajectories([robot])
        self.plot.drawRobots([robot])
        self.plot.draw()

        self.terminal.clearUpTo(2)
        self.terminal.println("Use a-s-d-w to navigate...")
        self.terminal.noecho()

        dAngle = 5.0 * pi / 180.0
        dForward = 0.1

        while True:
            key = self.terminal.keyPress()

            if key == 100:
                # right
                robot.move([0, -dAngle])
                commands.append([0, -dAngle])

            elif key == 97:
                # left
                robot.move([0, dAngle])
                commands.append([0, dAngle])

            elif key == 119:
                # up
                robot.move([dForward, 0])
                commands.append([dForward, 0])

            elif key == 115:
                # backward
                robot.move([-dForward, 0])
                commands.append([-dForward, 0])
            else:
                if self.terminal.yesno("Done?"):
                    self.terminal.clearUpTo(2)
                    break

            self.plot.clear()
            self.plot.drawMap(self.map)
            self.plot.drawRobotTrajectories([robot])
            self.plot.drawRobots([robot])
            self.plot.draw()

        self.plot.stop("Record Trajectory")
        self.terminal.clearUpTo(2)
        self.terminal.echo()

        # clean up the trajectory
        # if we turn, combine that with a forward motion
        cleaned = []
        accumulateTurn = 0
        for i in range(0, len(commands)):
            if commands[i][0] == 0:
                accumulateTurn = accumulateTurn + commands[i][1]
            else:
                cmd = commands[i]
                cmd[1] = accumulateTurn
                accumulateTurn = 0
                cleaned.append(cmd)

        trajName = self.terminal.queryForString("Please name this trajectory:")
        self.saveTrajectory(cleaned, trajName)

    def saveTrajectory(self, traj, name):
        if not os.path.exists('trajectories/'):
            os.makedirs('trajectories/')
        pickle.dump(traj, open('trajectories/' + name + ".p", "wb"))

    def stepThroughOneTrajectory(self, trajName):
        traj = self.loadTrajectory(trajName)

        if not traj:
            return False

        robot = self.robots[0]

        # r.sense(m.landmarks)
        # r.drawIdealObs(fig)
        self.plot.start("Step Through Trajectory")
        self.plot.drawMap(self.map)
        self.plot.drawRobotTrajectories([robot])
        self.plot.drawRobots([robot])
        self.plot.draw()

        self.terminal.clearUpTo(2)
        self.terminal.println("Press any key to step...")
        self.terminal.noecho()

        for step in traj:
            key = self.terminal.keyPress()

            robot.move(step)

            # r.sense(m.landmarks)
            # r.drawIdealObs(fig)
            self.plot.clear()
            self.plot.drawMap(self.map)
            self.plot.drawRobotTrajectories([robot])
            self.plot.drawRobots([robot])
            self.plot.draw()

        self.plot.stop("Step Through Trajectory")
        self.terminal.clearUpTo(2)
        self.terminal.echo()
        return True

    def loadTrajectory(self, name):
        if os.path.isfile("trajectories/" + name + ".p"):
            return pickle.load(open("trajectories/" + name + ".p", "rb"))
        else:
            return False
