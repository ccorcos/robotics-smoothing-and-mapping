from pylab import *
import os
import pickle

from Plot import *

from CommandLineApp import *
from utils import *


class Simulator:

    def __init__(self, simMap, robots):
        self.simMap = simMap
        self.robots = robots
        self.plot = Plot()

    def recordTrajectory(self, motion, initialPosition):
        """
        record a trajectory (sequence of commands) for a specific motion model
        """

        terminal = CommandLineApp()

        commands = []
        traj = [initialPosition[0:2]]
        pos = initialPosition

        # start an interactive plot
        self.plot.start("Record Trajectory")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(traj)
        self.plot.drawRobot(pos)
        self.plot.draw()

        terminal.clearUpTo(2)
        terminal.println("Use a-s-d-w to navigate...")
        terminal.noecho()

        dAngle = 5.0 * pi / 180.0
        dForward = 0.1

        while True:
            key = terminal.keyPress()

            if key == 100:
                # right
                pos = motion.move(pos, [0, -dAngle])
                commands.append([0, -dAngle])
                

            elif key == 97:
                # left
                pos = motion.move([0, dAngle])
                commands.append([0, dAngle])
                traj.append(pos)

            elif key == 119:
                # up
                pos = motion.move([dForward, 0])
                commands.append([dForward, 0])
                traj.append(pos)

            elif key == 115:
                # backward
                pos = motion.move([-dForward, 0])
                commands.append([-dForward, 0])
                traj.append(pos)

            else:
                if terminal.yesno("Done?"):
                    terminal.clearUpTo(2)
                    break

            # re-draw the updated plot
            self.plot.clear()
            self.plot.drawMap(self.simMap)
            self.plot.drawTrajectory(traj)
            self.plot.drawRobot(pos)
            self.plot.draw()
            
        # finish the plot
        self.plot.clear()
        self.plot.stop("Record Trajectory")
        # close curses
        terminal.clearUpTo(2)
        terminal.echo()
        terminal.doneCurses()

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

        trajName = raw_input("Please name this trajectory: ")
        self.saveTrajectory(cleaned, trajName)

    def saveTrajectory(self, traj, name):
        if not os.path.exists('trajectories/'):
            os.makedirs('trajectories/')
        pickle.dump(traj, open('trajectories/' + name + ".p", "wb"))

    def stepThroughTrajectory(self, trajName, motion, initialPosition):
        terminal = CommandLineApp()

        commands = self.loadTrajectory(trajName)

        if not traj:
            return False

        idealTraj = [initialPosition[0:2]]
        idealPos = initialPosition
        realTraj = [initialPosition[0:2]]
        realPos = initialPosition

        # start an interactive plot
        self.plot.start("Step Through Trajectory")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(idealTraj, "blue")
        self.plot.drawTrajectory(realTraj, "red")
        self.plot.drawRobot(idealPos, "blue")
        self.plot.drawRobot(realPos, "red")
        self.plot.draw()

        terminal.clearUpTo(2)
        terminal.println("Press any key to step...")
        terminal.noecho()

        for cmd in commands:
            key = terminal.keyPress()

            idealPos = motion.move(idealPos, cmd)
            realPos = motion.move(realPos, cmd)
            idealTraj.append(idealPos)
            realTraj.append(realPos)

            # update plot
            self.plot.clear()
            self.plot.drawMap(self.simMap)
            self.plot.drawTrajectory(idealTraj, "blue")
            self.plot.drawTrajectory(realTraj, "red")
            self.plot.drawRobot(idealPos, "blue")
            self.plot.drawRobot(realPos, "red")
            self.plot.draw()

        self.plot.clear()
        self.plot.stop("Step Through Trajectory")
        terminal.clearUpTo(2)
        terminal.echo()
        terminal.doneCurses()
        return True

    def loadTrajectory(self, name):
        if os.path.isfile("trajectories/" + name + ".p"):
            return pickle.load(open("trajectories/" + name + ".p", "rb"))
        else:
            return False

    # def runRobotThoughTrajectory(self, robot, traj):
    #     robot.reset()
    #     pos = [robot.state[0]['pos']]
    #     robot.simSense(self.simMap, pos[-1])
    #     for step in traj:
    #         pos.append(robot.simMove(pos[-1], step))
    #         robot.simSense(self.simMap, pos[-1])
    #     return pos

    