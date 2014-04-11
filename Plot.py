
from pylab import *
from random import gauss
import colorsys
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


class Plot:

    # def __init__(self):
    #     import colorsys
    #     import matplotlib.pyplot as plt
    #     from matplotlib.patches import Ellipse

    def start(self, name):
        plt.ion()
        self.fig = plt.figure(name)
        self.ax = subplot(111)

    def draw(self):
        self.fig.canvas.draw()

    def clear(self):
        self.ax.cla()

    def stop(self, name):
        self.ax.cla()
        plt.ioff()
        plt.close(name)

    def drawMap(self, simMap, types):
        for key in simMap.plotLandmarks:
            lms = np.array(simMap.plotLandmarks[key])
            self.ax.scatter(lms[:, 0],
                            lms[:, 1],
                            color=types[key]["color"],
                            marker=types[key]["marker"],
                            s=types[key]["scale"])

        # for l in simMap.landmarks:
        #     self.ax.scatter(l['pos'][0],
        #                     l['pos'][1],
        #                     color=types[l["type"]]["color"],
        #                     marker=types[l["type"]]["marker"],
        #                     s=types[l["type"]]["scale"])

    def drawRobotMap(self, robot, types):
        for key in robot.plotLandmarks:
            lms = np.array(robot.plotLandmarks[key])
            self.ax.scatter(lms[:, 0],
                            lms[:, 1],
                            color=types[key]["color"],
                            marker=types[key]["marker"],
                            s=types[key]["scale"])

    def drawRobotTrajectory(self, robot, color):
        traj = robot.trajectoryXY()
        self.drawTrajectory(traj, color)

    def drawTrajectory(self, traj, color):
        traj = np.array(traj)
        self.ax.plot(traj[:, 0],
                     traj[:, 1],
                     color=color)

    def drawRobotObservation(self, robot, pos, colors):
        st = robot.state[-1]

        xy = np.array(pos[0:2])
        a = np.array(pos[2])
        for ob in st['obs']:
            d = ob["sensor pos"][0]
            t = ob["sensor pos"][1]

            lm = np.array([d * cos(a + t), d * sin(a + t)])

            arrow = np.array([xy, xy + lm])

            self.ax.plot(arrow[:, 0],
                         arrow[:, 1],
                         color=colors[ob["type"]])

            sensor = robot.sensorOfType(ob["type"])

            distErr = sensor.noise[0] * 4
            angleErr = d * sensor.noise[1] * 4

            e = Ellipse(xy=xy + lm,
                        width=distErr,
                        height=angleErr,
                        angle=t * 180 / pi,
                        alpha=0.5,
                        color=colors[ob["type"]])
            self.ax.add_artist(e)

    def drawRobot(self, pos, color):
        self.ax.scatter(pos[0],
                        pos[1],
                        s=50,
                        marker="o",
                        color=color)
        a = pos[2]
        head = np.array([cos(a), sin(a)]) * 0.2
        start = np.array(pos[0:2])
        end = start + head
        pts = np.array([start, end])
        self.ax.plot(pts[:, 0],
                     pts[:, 1],
                     color=color)
