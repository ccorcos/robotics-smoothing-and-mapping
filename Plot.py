
from pylab import *
import colorsys
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

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

    def drawMap(self, simMap):
        types = simMap.landmarkTypes
        
        cm = get_cmap('gist_rainbow')
        for i in range(len(types)):
            color = cm(1. * i / len(types))
            landmarkType = types[i]
            # filter for the landmarks of this type
            landmarks = filter(lambda x: x['type'] == landmarkType, simMap.landmarks)
            # get the landmark positions
            landmarks = map(lambda x: x['pos'], simMap.landmarks)
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
                     color=color)

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

    # def drawRobotMap(self, robot, types):
    #     for key in robot.plotLandmarks:
    #         lms = np.array(robot.plotLandmarks[key])
    #         self.ax.scatter(lms[:, 0],
    #                         lms[:, 1],
    #                         color=types[key]["color"],
    #                         marker=types[key]["marker"],
    #                         s=types[key]["scale"])

    # def drawRobotTrajectory(self, robot, color):
    #     traj = robot.trajectoryXY()
    #     self.drawTrajectory(traj, color)

    

    # def drawRobotObservation(self, robot, pos, colors):
    #     st = robot.state[-1]

    #     xy = np.array(pos[0:2])
    #     a = np.array(pos[2])
    #     for ob in st['obs']:
    #         d = ob["sensor pos"][0]
    #         t = ob["sensor pos"][1]

    #         lm = np.array([d * cos(a + t), d * sin(a + t)])

    #         arrow = np.array([xy, xy + lm])

    #         self.ax.plot(arrow[:, 0],
    #                      arrow[:, 1],
    #                      color=colors[ob["type"]])

    #         sensor = robot.sensorOfType(ob["type"])

    #         distErr = sensor.noise[0] * 4
    #         angleErr = d * sensor.noise[1] * 4

    #         e = Ellipse(xy=xy + lm,
    #                     width=distErr,
    #                     height=angleErr,
    #                     angle=t * 180 / pi,
    #                     alpha=0.5,
    #                     color=colors[ob["type"]])
    #         self.ax.add_artist(e)

    
