
# todo
# plot ellipses, size and angle
# plot real vs ideal
# unify measurements and error, etc. Clean up! Python Packages!
# modularize
# SAM
# iSAM
# python package
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
from Robot import *
from Map import *

sensorOptions = {
    'type': '1',
    'maxDistance': 1,
    'maxAngle': pi,
    'distanceNoise': 0.01,  # 1 cm stdev => within 4cm 95% of the time
    # 5 degrees stdev => within 20 degrees 95% of the time
    'angleNoise': 5 * pi / 180
}

s = LaserSensorSim(sensorOptions)

robotOptions = {
    'motionModel': unicycleModel,
    'initialPosition': [5, 5, 0],  # x, y, angle
    'motionNoise': [0.01, 1 * pi / 180],  # forward, turn
    'sensors': [s]
}

r = Robot(robotOptions)

mapOptions = {
    "scale": 10,
    "landmarkTypes": [{"type": '1',
                       "n": 200}]
}

m = Map(mapOptions)


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

    if terminal.yesno("Would you like to step through a trajectory?"):
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
