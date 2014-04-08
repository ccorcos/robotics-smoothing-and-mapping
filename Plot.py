
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

    def get_color(self, colors, idx=0):
        for hue in range(colors):
            hue = 1. * hue / colors
            col = [int(x) for x in colorsys.hsv_to_rgb(hue, 1.0, 230)]
            if idx == 0:
                yield "#{0:02x}{1:02x}{2:02x}".format(*col)
            elif idx == 1:
                yield "#{2:02x}{1:02x}{0:02x}".format(*col)
            elif idx == 2:
                yield "#{1:02x}{0:02x}{2:02x}".format(*col)
            else:  # idx == 3
                yield "#{2:02x}{0:02x}{1:02x}".format(*col)

    def drawRobotTrajectory(self, robot, color):
        # color = self.get_color(len(robots), idx=1)
        # for robot in robots:
            # acolor = next(color)
        traj = robot.trajectory()
        self.drawTrajectory(traj, color)
            # self.ax.plot(traj[:, 0],
            #              traj[:, 1],
            #              color=acolor)

    def drawTrajectory(self, traj, color):
        traj = np.array(traj)
        self.ax.plot(traj[:, 0],
                     traj[:, 1],
                     color=color)

    def drawRobotObservation(self, robot, pos, color):
        sensorTypes = []
        for sensor in robot.sensors:
            sensorTypes.append(sensor.type)

        sensorTypes = list(set(sensorTypes))

        st = robot.state[-1]
        # color = self.get_color(len(sensorTypes), idx=2)
        for sense in sensorTypes:
            # acolor = next(color)
            if sense in st.obs:
                sensor = robot.sensorOfType(sense)
                for ob in st.obs[sense]:
                    xy = np.array(pos[0:2])
                    a = np.array(pos[2])
                    d = ob["where"][0]
                    t = ob["where"][1]
                    # lm = np.array(ob["where"])

                    lm = np.array([d * cos(a + t), d * sin(a + t)])

                    arrow = np.array([xy, xy + lm])
                    self.ax.plot(arrow[:, 0],
                                 arrow[:, 1],
                                 color=color)

                    angleErr = d * sensor.angleNoise * 4
                    distErr = sensor.distanceNoise * 4
                    # ang = dot(p - o, [1, 0]) / d

                    e = Ellipse(xy=xy + lm,
                                width=distErr,
                                height=angleErr,
                                angle=t * 180 / pi,
                                alpha=0.5,
                                color=color)
                    self.ax.add_artist(e)

    def drawRobot(self, pos, color):
        # color = self.get_color(len(robots), idx=1)
        # for robot in robots:
            # acolor = next(color)
        # state = robot.state[-1]
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
