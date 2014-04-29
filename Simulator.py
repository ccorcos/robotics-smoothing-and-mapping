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
                traj.append(pos[0:2])
                

            elif key == 97:
                # left
                pos = motion.move(pos, [0, dAngle])
                commands.append([0, dAngle])
                traj.append(pos[0:2])

            elif key == 119:
                # up
                pos = motion.move(pos, [dForward, 0])
                commands.append([dForward, 0])
                traj.append(pos[0:2])

            elif key == 115:
                # backward
                pos = motion.move(pos, [-dForward, 0])
                commands.append([-dForward, 0])
                traj.append(pos[0:2])

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

        if not commands:
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
            realPos = motion.simMove(realPos, cmd)

            idealTraj.append(idealPos[0:2])
            realTraj.append(realPos[0:2])

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

    def stepThroughRobotGraph(self, robot, trajName):
        
        terminal = CommandLineApp()

        commands = self.loadTrajectory(trajName)

        if not commands:
            return False

        robot.reset()
        initialPosition = robot.position()
        
        # the real positions of the robot
        positions = [initialPosition]
        pos = initialPosition

        robot.simSense(self.simMap, pos)

        # start an interactive plot
        self.plot.start("Building Robot Graph")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(positions, "blue")
        self.plot.drawRobotGraph(robot, "green", 0.2)
        self.plot.drawRobotTrajectory(robot, "red")

        self.plot.draw()

        terminal.clearUpTo(2)
        terminal.println("Press any key to step...")
        terminal.noecho()

        for cmd in commands:
            key = terminal.keyPress()

            # move the robot. the robot updates its graph
            robot.move(cmd)
            # update where the robot really is due to errors
            pos = robot.motion.simMove(pos, cmd)
            positions.append(pos)
            # sense
            robot.simSense(self.simMap, pos)

            # update plot
            self.plot.clear()
            self.plot.drawMap(self.simMap)
            self.plot.drawTrajectory(positions, "blue")
            self.plot.drawRobotGraph(robot, "green", 0.2)
            self.plot.drawRobotTrajectory(robot, "red")
            self.plot.draw()

        self.plot.clear()
        self.plot.stop("Building Robot Graph")
        terminal.clearUpTo(2)
        terminal.echo()
        terminal.doneCurses()
        return True

    def stepThroughRobotObservations(self, robot, trajName):
        
        terminal = CommandLineApp()

        commands = self.loadTrajectory(trajName)

        if not commands:
            return False

        robot.reset()
        initialPosition = robot.position()
        
        # the real positions of the robot
        positions = [initialPosition]
        pos = initialPosition

        robot.simSense(self.simMap, pos)

        # start an interactive plot
        self.plot.start("Drawing robot trajectory and Observations")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(positions, "blue")
        self.plot.drawRobotTrajectory(robot, "red")
        self.plot.drawRobotObservations(robot, "green", 0.2)
        self.plot.draw()

        terminal.clearUpTo(2)
        terminal.println("Press any key to step...")
        terminal.noecho()

        for cmd in commands:
            key = terminal.keyPress()

            # move the robot. the robot updates its graph
            robot.move(cmd)
            # update where the robot really is due to errors
            pos = robot.motion.simMove(pos, cmd)
            positions.append(pos)
            # sense
            robot.simSense(self.simMap, pos)

            # update plot
            self.plot.clear()
            self.plot.drawMap(self.simMap)
            self.plot.drawTrajectory(positions, "blue")
            self.plot.drawRobotTrajectory(robot, "red")
            self.plot.drawRobotObservations(robot,"green", 0.2)
            self.plot.draw()

        self.plot.clear()
        self.plot.stop("Building Robot Graph")
        terminal.clearUpTo(2)
        terminal.echo()
        terminal.doneCurses()
        return True

    def stepThroughRobotMotion(self, robot, trajName):
        
        terminal = CommandLineApp()

        commands = self.loadTrajectory(trajName)

        if not commands:
            return False

        robot.reset()
        initialPosition = robot.position()
        
        # the real positions of the robot
        positions = [initialPosition]
        pos = initialPosition

        robot.simSense(self.simMap, pos)

        # start an interactive plot
        self.plot.start("Drawing robot trajectory and Observations")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(positions, "blue")
        self.plot.drawRobotTrajectory(robot, "red")
        self.plot.drawRobotMotion(robot, "green", 1)
        self.plot.draw()

        terminal.clearUpTo(2)
        terminal.println("Press any key to step...")
        terminal.noecho()

        for cmd in commands:
            key = terminal.keyPress()

            # move the robot. the robot updates its graph
            robot.move(cmd)
            # update where the robot really is due to errors
            pos = robot.motion.simMove(pos, cmd)
            positions.append(pos)
            # sense
            robot.simSense(self.simMap, pos)

            # update plot
            self.plot.clear()
            self.plot.drawMap(self.simMap)
            self.plot.drawTrajectory(positions, "blue")
            self.plot.drawRobotTrajectory(robot, "red")
            self.plot.drawRobotMotion(robot,"green", 1)
            self.plot.draw()

        self.plot.clear()
        self.plot.stop("Building Robot Graph")
        terminal.clearUpTo(2)
        terminal.echo()
        terminal.doneCurses()
        return True

    def stepThroughRobotObservationsAndMotion(self, robot, trajName):
        
        terminal = CommandLineApp()

        commands = self.loadTrajectory(trajName)

        if not commands:
            return False

        robot.reset()
        initialPosition = robot.position()
        
        # the real positions of the robot
        positions = [initialPosition]
        pos = initialPosition

        robot.simSense(self.simMap, pos)

        # start an interactive plot
        self.plot.start("Drawing robot trajectory and Observations")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(positions, "blue")
        self.plot.drawRobotTrajectory(robot, "red")
        self.plot.drawRobotMotion(robot, "green", 0.2)
        self.plot.drawRobotObservations(robot, "green", 0.2)
        self.plot.draw()

        terminal.clearUpTo(2)
        terminal.println("Press any key to step...")
        terminal.noecho()

        for cmd in commands:
            key = terminal.keyPress()

            # move the robot. the robot updates its graph
            robot.move(cmd)
            # update where the robot really is due to errors
            pos = robot.motion.simMove(pos, cmd)
            positions.append(pos)
            # sense
            robot.simSense(self.simMap, pos)

            # update plot
            self.plot.clear()
            self.plot.drawMap(self.simMap)
            self.plot.drawTrajectory(positions, "blue")
            self.plot.drawRobotTrajectory(robot, "red")
            self.plot.drawRobotMotion(robot,"green", 0.2)
            self.plot.drawRobotObservations(robot, "green", 0.2)
            self.plot.draw()

        self.plot.clear()
        self.plot.stop("Building Robot Graph")
        terminal.clearUpTo(2)
        terminal.echo()
        terminal.doneCurses()
        return True

    def stepThroughSAM(self, robot, trajName):
        
        pr(0, "Step Through SAM")

        commands = self.loadTrajectory(trajName)

        if not commands:
            return False

        pr(1, "loaded trajectory", trajName)

        pr(1, "running trajectory")

        robot.reset()
        initialPosition = robot.position()
        
        # the real positions of the robot
        positions = [initialPosition]
        pos = initialPosition

        
        robot.simSense(self.simMap, pos)
        
        for cmd in commands:

            # move the robot. the robot updates its graph
            robot.move(cmd)
            # update where the robot really is due to errors
            pos = robot.motion.simMove(pos, cmd)
            positions.append(pos)
            # sense
            robot.simSense(self.simMap, pos)

        pr(1, "trajectory finished")
        pr(2, len(robot.graph.nodes) ,"nodes")
        pr(2, len(robot.graph.edges) ,"edges")

        pr(1, "plotting graph")

        self.plot.new("Robot Graph")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(positions, "blue")
        self.plot.drawRobotTrajectory(robot, "red")
        self.plot.drawRobotGraph(robot, "yellow", 0.8)
        self.plot.drawRobotObservations(robot, "green", 0.2)
        self.plot.drawRobotAngle(robot,"orange")
        self.plot.show()
        
        wait()
        
        beta = 0.2
        maxIter = 20
        minMean = 0.05
        pr(1, "optimization step", 0)
        m = robot.graph.optimizationStep(beta)    
        pr(1, "dx mean", m)

        iteration = 1
        while iteration < maxIter and m > minMean:

            pr(1, "optimization step", iteration)
            m = robot.graph.optimizationStep(beta)    
            pr(1, "dx mean", m) 
            iteration = iteration + 1

        pr(1, "done")
        self.plot.new("Robot Graph")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(positions, "blue")
        self.plot.drawRobotTrajectory(robot, "red")
        self.plot.drawRobotGraph(robot, "yellow", 0.8)
        self.plot.drawRobotObservations(robot, "green", 0.2)
        self.plot.drawRobotAngle(robot,"orange")
        self.plot.show()

        return True

    def stepIncrementalSAM(self, robot, trajName):
        
        pr(0, "Step Through SAM")

        commands = self.loadTrajectory(trajName)

        if not commands:
            return False

        pr(1, "loaded trajectory", trajName)

        pr(1, "running trajectory")

        robot.reset()
        initialPosition = robot.position()
        
        # the real positions of the robot
        positions = [initialPosition]
        pos = initialPosition

        batch = 50

        i = 0
        robot.simSense(self.simMap, pos)
        
        for cmd in commands:
            i = i+1
            # move the robot. the robot updates its graph
            robot.move(cmd)
            # update where the robot really is due to errors
            pos = robot.motion.simMove(pos, cmd)
            positions.append(pos)
            # sense
            robot.simSense(self.simMap, pos)
            if i == batch:
                i = 0
                robot.graph.optimize(0.2)
                self.plot.new("Robot Graph")
                self.plot.drawMap(self.simMap)
                self.plot.drawTrajectory(positions, "blue")
                self.plot.drawRobotTrajectory(robot, "red")
                self.plot.drawRobotGraph(robot, "yellow", 0.8)
                self.plot.drawRobotObservations(robot, "green", 0.2)
                self.plot.drawRobotAngle(robot,"orange")
                self.plot.show()
                wait()

        pr(1, "last")
        robot.graph.optimize(0.2)
        pr(1, "done")
        self.plot.new("Robot Graph")
        self.plot.drawMap(self.simMap)
        self.plot.drawTrajectory(positions, "blue")
        self.plot.drawRobotTrajectory(robot, "red")
        self.plot.drawRobotGraph(robot, "yellow", 0.8)
        self.plot.drawRobotObservations(robot, "green", 0.2)
        self.plot.drawRobotAngle(robot,"orange")
        self.plot.show()

        return True