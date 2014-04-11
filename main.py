
# todo
# action model and observation model function, linearizaion, and jacobian
#
# SAM
# iSAM
#
# python package
#
# multirobot
# multisensor
# decentralized?


# create simulation data
#   create a 10 by 10 map with 200 landmarks
#   create a robot in the middle of the map
#       random walk
#       motion model
#       sensor model

from CommandLineApp import *
terminal = CommandLineApp()

from pylab import *
from random import gauss

from Simulator import *
from Sensor import *
from Motion import *
from Robot import *
from Map import *

mapOptions = {
    "scale": 10,
    "landmarkTypes": [{"type": '1',
                       "n": 200}]
}

m = Map(mapOptions)

sensorOptions = {
    'type': '1',
    'maxDistance': 1,
    'maxAngle': pi,
    'noise': [0.01, 5 * pi / 180]  # distance, angle
    # 1 cm stdev => within 4cm 95% of the time
    # 5 degrees stdev => within 20 degrees 95% of the time
}

s = LaserSensorSim(sensorOptions)

motionOptions = {
    'noise': [0.01, 1 * pi / 180],  # forward, turn
}

u = UnicycleModel(motionOptions)

robotOptions = {
    'motionModel': u,
    'sensors': [s],
    'initialPosition': [5, 5, 0]  # x, y, angle
}

r = Robot(robotOptions)


simulatorOptions = {
    "map": m,
    "robots": [r],
    "terminal": terminal
}

sim = Simulator(simulatorOptions)


def main():

    terminal.println("Chet's SAM Algorithm")
    terminal.nextLine()

    if terminal.yesno("Would you to record a trajectory?"):
        sim.recordTrajectory()

    if terminal.yesno("Would you like to step through an ideal trajectory?"):
        traj = False
        while not traj:
            trajName = terminal.queryForString("which trajectory:")
            traj = sim.stepThroughOneTrajectory(trajName)

    if terminal.yesno("Would you like to step through a real trajectory?"):
        traj = False
        while not traj:
            trajName = terminal.queryForString("which trajectory:")
            traj = sim.stepThroughRealTrajectory(trajName)


if __name__ == "__main__":
    terminal.start(main)
