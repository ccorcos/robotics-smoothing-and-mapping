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
landmarkTypes = [{"type":'laser1', "n": 20}, {"type":'laser2', "n": 10}, {"type":'laser3', "n": 10}]
mapScale = 10
simMap = Map(mapScale, landmarkTypes)

# create a sensor to sense the landmarks on the map
# 1 cm stdev => within 4cm 95% of the time
# 5 degrees stdev => within 20 degrees 95% of the time
# senseNoise = [0.01, 1 * pi / 180]  # distance, angle
# senseNoise = [0.001, 0.1 * pi / 180]  # distance, angle
senseNoise = [0.01, 1 * pi / 180]  # distance, angle
maxAngle = pi/2. # this sensor can go 360 degrees
maxDistance = 3 # 1 meter on a 10 by 10 map
sensorType = "laser1"
sensor1 = LaserSensorSim(senseNoise, sensorType, maxDistance, maxAngle)

senseNoise = [0.5, 10 * pi / 180]  # distance, angle
maxAngle = pi/4. # this sensor can go 360 degrees
maxDistance = 8 # 1 meter on a 10 by 10 map
sensorType = "laser2"
sensor2= LaserSensorSim(senseNoise, sensorType, maxDistance, maxAngle)

senseNoise = [0.001, 0.1 * pi / 180]  # distance, angle
maxAngle = pi/4. # this sensor can go 360 degrees
maxDistance = 10 # 1 meter on a 10 by 10 map
sensorType = "laser3"
sensor3= LaserSensorSim(senseNoise, sensorType, maxDistance, maxAngle)



# this robot will move like a unicycle
# motionNoise = [0.01, 1 * pi / 180]  # forward, turn 
# motionNoise = [0.001, .1 * pi / 180]  # forward, turn 
motionNoise = [0.05, 2 * pi / 180]  # forward, turn 
unicycle1 = UnicycleModel(motionNoise)


# create the robot
initialPosition1 = [8, 8, -2]  # x, y, angle
robot1 = Robot( [sensor1, sensor3], unicycle1, initialPosition1, "robot1")


initialPosition2 = [2, 2, 1]  # x, y, angle
motionNoise = [0.1, 5 * pi / 180]  # forward, turn 
unicycle2 = UnicycleModel(motionNoise)
robot2 = Robot( [sensor1, sensor2], unicycle2, initialPosition2, "robot2")


sim = Simulator(simMap, [robot1, robot2])


def recordTraj():
    if yesno("Would you to record a trajectory?: "):
        sim.recordTrajectory(unicycle, initialPosition)

def stepTraj():
    if yesno("Would you like to step through a trajectory?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughTrajectory(trajName, unicycle, initialPosition)
        if not traj:
            stepTraj()

def stepRobotGraph():
    if yesno("Would you like to step through the robot graph?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughRobotGraph(robot, trajName)
        if not traj:
            stepRobotGraph()

def stepRobotObservations():
    if yesno("Would you like to step through the robot observations?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughRobotObservations(robot, trajName)
        if not traj:
            stepRobotObservations()

def stepRobotMotion():
    if yesno("Would you like to step through the robot motion?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughRobotMotion(robot, trajName)
        if not traj:
            stepRobotMotion()

def stepRobotObservationsAndMotion():
    if yesno("Would you like to step through the robot observations and motion?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughRobotObservationsAndMotion(robot, trajName)
        if not traj:
            stepRobotObservationsAndMotion()

def stepRobotSAM():
    if yesno("Would you like to step through SAM?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepThroughSAM(robot, trajName)
        if not traj:
            stepRobotSAM()

def stepIncrementalSAM():
    if yesno("Would you like to step through incremental SAM?"):
        trajName = raw_input("which trajectory: ")
        traj = sim.stepIncrementalSAM(robot, trajName)
        if not traj:
            stepIncrementalSAM()


def main():

    pr(0,"Chet's SAM Algorithm")

    # print unicycle.jacobianPosition([0,0,1],[1,1])
    # print sensor.jacobianPosition([0,0,0],[1,1])
    # print sensor.jacobianLandmark([0,0,0],[1,1])
    # wait()

    # recordTraj()
    # stepTraj()
    # stepRobotGraph()
    # stepRobotObservations()
    # stepRobotMotion()
    # stepRobotObservationsAndMotion()
    # stepRobotSAM()
    # stepIncrementalSAM()
    

    # multiple robot SAM
    # pr(1,"recording both robot trajectories")
    # sim.recordTrajectory(unicycle1, initialPosition1)
    # sim.recordTrajectory(unicycle2, initialPosition2)
    
    robot1.graph.addNode(robot2.graph.nodes[0])
    robot1.graph.addEdge(robot2.graph.edges[0])
    robot2.graph = robot1.graph

    sim.stepMultiRobotIncrementalSAM([robot1, robot2], ["topright", "bottomleft"])

    
    # sim.stepThroughRobotObservationsAndMotion(robot, "quick")
    # sim.stepThroughSAM(robot, "quick")


if __name__ == "__main__":
    main()
