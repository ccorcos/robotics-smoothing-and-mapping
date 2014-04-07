
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

    def drawMap(self, m):
        color = self.get_color(len(m.landmarkTypes))
        for lmType in m.landmarkTypes:
            acolor = next(color)
            self.ax.scatter(
                m.landmarks[lmType][:, 0],
                m.landmarks[lmType][:, 1],
                color=acolor,
                marker="x")

    def get_color(self, colors, reverse=0):
        for hue in range(colors):
            hue = 1. * hue / colors
            col = [int(x) for x in colorsys.hsv_to_rgb(hue, 1.0, 230)]
            if reverse:
                yield "#{2:02x}{1:02x}{0:02x}".format(*col)
            else:
                yield "#{0:02x}{1:02x}{2:02x}".format(*col)

    def drawRobotTrajectories(self, robots):
        color = self.get_color(len(robots), reverse=1)
        for robot in robots:
            acolor = next(color)
            traj = robot.trajectory()
            self.ax.plot(traj[:, 0],
                         traj[:, 1],
                         color=acolor)

    def drawRobots(self, robots):
        color = self.get_color(len(robots), reverse=1)
        for robot in robots:
            acolor = next(color)
            state = robot.state[-1]
            self.ax.scatter(state.pos[0],
                            state.pos[1],
                            s=50,
                            marker="o",
                            color=acolor)
            a = state.pos[2]
            head = np.array([cos(a), sin(a)]) * 0.2
            start = np.array(state.pos[0:2])
            end = start + head
            pts = np.array([start, end])
            self.ax.plot(pts[:, 0],
                         pts[:, 1],
                         color=acolor)
