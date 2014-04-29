
from pylab import *
import colorsys
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# 
# blue = real trajectory
# red = robot, dead reckoned trajectory

def colorAlpha(name):
    cm = get_cmap('gist_rainbow')
    if name == "laser1":
        color = cm(1. / 3.)
        alpha = 0.5
        zorder = -1
        return color, alpha, zorder
    elif name == "laser2":
        color = cm(2. / 3.)
        alpha = 0.01
        zorder = -10000000
        return color, alpha, zorder
    elif name == "laser3":
        color = cm(3. / 3.)
        alpha = 0.1
        zorder = -1
        return color, alpha, zorder


class Plot:

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

    def new(self, name=""):
        self.fig = plt.figure(name)
        self.ax = subplot(111)

    def show(self):
        plt.show()

    def drawMap(self, simMap):
        types = simMap.landmarkTypes
        # cm = get_cmap('gist_rainbow')
        for i in range(len(types)):
            # color = cm(1. * i / float(len(types)))
            landmarkType = types[i]
            color, alpha, zorder = colorAlpha(landmarkType)
            # filter for the landmarks of this type
            landmarks = filter(lambda x: x['type'] == landmarkType, simMap.landmarks)
            # get the landmark positions
            landmarks = map(lambda x: x['pos'], landmarks)
            landmarks = array(landmarks)
            self.ax.scatter(landmarks[:, 0],
                            landmarks[:, 1],
                            color=color,
                            marker='x',
                            s=5)

    def drawTrajectory(self, traj, color="red"):
        traj = np.array(traj)
        self.ax.plot(traj[:, 0],
                     traj[:, 1],
                     color=color,
                     zorder=10000000)

    def drawRobotTrajectory(self, robot, color="red"):
        traj = robot.trajectoryXY()
        self.drawTrajectory(traj, color=color)

    def drawRobotAngle(self, robot, color="red"):
        traj = robot.trajectory()
        for pos in traj:
            a = pos[2]
            head = array([cos(a), sin(a)]) * 0.2
            start = array(pos[0:2])
            end = start + head
            pts = array([start, end])
            self.ax.plot(pts[:, 0],
                         pts[:, 1],
                         color=color)

    def drawRobotMotion(self, robot, color="red", alpha= 0.5):
        edges = robot.graph.getEdgesOfType("motion")
        
        for edge in edges:
            pos0 = array(edge.node1.value)
            pos1 = array(edge.node2.value)
            cmd = edge.value
            f = cmd[0]
            t = cmd[1]

            pos11 = edge.model.move(pos0, cmd)


            xy0 = pos0[0:2]
            xy11 = pos11[0:2]
            a0 = pos0[2]
            a11 = pos11[2]

            arrow = np.array([xy0, xy11])

            self.ax.plot(arrow[:, 0],
                         arrow[:, 1],
                         color=color,
                         alpha=alpha)

            motion = edge.model
            forwardErr = motion.noise[0] * 4
            turnErr = f * motion.noise[1] * 4

            e = Ellipse(xy=xy11,
                        width=forwardErr,
                        height=turnErr,
                        angle=(a11)  * 180 / pi,
                        alpha=alpha,
                        color=color)
            self.ax.add_artist(e)


    def drawRobot(self, pos, color="red"):
        self.ax.scatter(pos[0],
                        pos[1],
                        s=50,
                        marker="o",
                        color=color)
        a = pos[2]
        head = array([cos(a), sin(a)]) * 0.2
        start = array(pos[0:2])
        end = start + head
        pts = array([start, end])
        self.ax.plot(pts[:, 0],
                     pts[:, 1],
                     color=color)

    def drawRobotGraph(self, robot, color="red", alpha=0.5):
        traj = robot.trajectoryXY()
        self.drawTrajectory(traj)
        for edge in robot.graph.edges:
            xy1 = edge.node1.value[0:2]
            xy2 = edge.node2.value[0:2]
            if edge.edgeType == "observation":
                arrow = np.array([xy1,xy2])
                self.ax.plot(arrow[:, 0],
                             arrow[:, 1],
                             color=color,
                             alpha=alpha,
                             zorder=-100000000000)

    def drawRobotObservations(self, robot, color="red", alpha=0.5):
        edges = robot.graph.getEdgesOfType("observation")
        
        for edge in edges:
            pos = array(edge.node1.value)
            lm = array(edge.model.deadReckon(pos, edge.value))
            xy = pos[0:2]
            a = pos[2]
            d = edge.value[0]
            t = edge.value[1]

            arrow = np.array([xy, lm])

            color, alpha, zorder = colorAlpha(edge.model.sensorType)

            self.ax.plot(arrow[:, 0],
                         arrow[:, 1],
                         color=color,
                         alpha=alpha,
                         zorder=zorder)

            sensor = edge.model
            distErr = sensor.noise[0] * 4
            angleErr = d * sensor.noise[1] * 4

            e = Ellipse(xy=lm,
                        width=distErr,
                        height=angleErr,
                        angle=(t+a)  * 180 / pi,
                        alpha=alpha,
                        color=color,
                        zorder=zorder)
            self.ax.add_artist(e)
        

    # def drawRobotMap(self, robot, types):
    #     for key in robot.plotLandmarks:
    #         lms = np.array(robot.plotLandmarks[key])
    #         self.ax.scatter(lms[:, 0],
    #                         lms[:, 1],
    #                         color=types[key]["color"],
    #                         marker=types[key]["marker"],
    #                         s=types[key]["scale"])

  