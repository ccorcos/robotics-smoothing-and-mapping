# -*- coding: utf-8 -*-
'''
Created on 2014-04-24 18:10
@summary: 
@author: chetcorcos
'''

from pylab import *

from Map import *
from Sensor import *
from Motion import *
from Robot import *
from Simulator import *

# create a 10 by 10 map with 200 landmarks
landmarkTypes = [{"type":'laser', "n": 200}]
mapScale = 10
simMap = Map(mapScale, landmarkTypes)

# create a sensor to sense the landmarks on the map
# 1 cm stdev => within 4cm 95% of the time
# 5 degrees stdev => within 20 degrees 95% of the time
senseNoise = [0.01, 5 * pi / 180]  # distance, angle
maxAngle = pi # this sensor can go 360 degrees
maxDistance = 1 # 1 meter on a 10 by 10 map
sensorType = "laser"
sensor = LaserSensorSim(senseNoise, sensorType, maxDistance, maxAngle)


# this robot will move like a unicycle
motionNoise = [0.01, 1 * pi / 180]  # forward, turn 
unicycle = UnicycleModel(motionNoise)


# create the robot
initialPosition = [5, 5, 0]  # x, y, angle
robot = Robot( [sensor], unicycle, initialPosition)

sim = Simulator(simMap, [robot])


def recordTraj():
    if yesno("Would you to record a trajectory?: "):
        sim.recordTrajectory(unicycle, initialPosition)

def stepTraj():
    if yesno("Would you like to step through an ideal trajectory?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughTrajectory(trajName, unicycle, initialPosition)
        if not traj:
            stepTraj()


def main():

    print "Chet's SAM Algorithm\n"

    recordTraj()
    stepTraj()

if __name__ == "__main__":
    main()
