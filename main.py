
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


from pylab import *
from random import gauss

from Simulator import *
from Sensor import *
from Motion import *
from Robot import *
from Map import *

mapOptions = {
    "scale": 10,
    "landmarkTypes": [{"type": 'type1',
                       "n": 200}],
}

m = Map(mapOptions)

sensorOptions = {
    'type': 'type1',
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
    "robots": [r]
}

sim = Simulator(simulatorOptions)


def recordTraj():
    if yesno("Would you to record a trajectory?: "):
        sim.recordTrajectory()


def stepIdeal():
    if yesno("Would you like to step through an ideal trajectory?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughOneTrajectory(trajName)
        if not traj:
            stepIdeal()


def stepReal():
    if yesno("Would you like to step through a real trajectory?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughRealTrajectory(trajName)
        if not traj:
            stepReal()


def nonlinearSAM():
    if yesno("Would you like to run nonlinear SAM on a trajectory?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.runNonlinearSAM(trajName)
        if not traj:
            nonlinearSAM()


def main():

    print "Chet's SAM Algorithm\n"

    # recordTraj()
    # stepIdeal()
    # stepReal()
    # nonlinearSAM()
    sim.runNonlinearSAM('circle')


if __name__ == "__main__":
    main()
