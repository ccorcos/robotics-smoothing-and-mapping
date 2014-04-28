# -*- coding: utf-8 -*-
'''
Created on 2014-04-24 17:55
@summary: Robot class for SAM simulation
@author: chetcorcos

robot.move()
    move,
    create a node,
    add edges,

robot.sense()
    sense,
    create nodes as necessary
    add edges
'''

from pylab import *
from utils import *
from Graph import *


class Robot:

    def __init__(self, sensors, motion, initialPosition):
        self.sensors = sensors
        self.motion = motion
        # initialize the graph
        self.graph = Graph()

        # create a first node for the initial position
        node = Node(initialPosition, "position", 0)
        self.graph.addNode(node)

        # add a prior
        edge = PriorEdge(initialPosition, node, np.diag([1e-20,1e-20,1e-20]))
        self.graph.addEdge(edge)

        # keep track of the most recent position for dead reakoning
        self.pos = initialPosition
        self.posNode = node # node of the current position
        

    def move(self, cmd):
        """create a new node and edge with dead reakoned initial value"""
        nextPos = self.motion.move(self.pos, cmd)
        # create a new node for the next position
        # the descriptor is the index in time
        nextPosNode = Node(nextPos, "position", self.posNode.descriptor + 1)
        self.graph.addNode(nextPosNode)
        # create a new edge between them
        edge = MotionEdge(self.motion, cmd, self.posNode, nextPosNode, "motion")
        self.graph.addEdge(edge)
        # update internal state
        self.pos = nextPos
        self.posNode = nextPosNode


    def simSense(self, simMap, simPos):
        """
        Simulate sensing landmarks on a map. The simulator gives the 
        robots actual position. 
        Create a new node for the landmark if necessay with a dead
        reakoned initial value with respect to the position node.
        """

        for sensor in self.sensors:
            sensorObsverations = sensor.simSense(simPos, simMap)

            for lmObs in sensorObsverations:
                # check if this landmark has been observed before
                lmNode = self.graph.getNodeOfTypeAndDescriptor(lmObs['sensorType'], lmObs['descriptor'])
                if lmNode == None: 
                    # if this landmark hasn't been observed before
                    # create a new node
                    lmPos = sensor.deadReckon(self.pos,lmObs['obs'])
                    # print self.pos
                    # print lmPos
                    # print lmObs['obs']
                    # wait()
                    lmNode = Node(lmPos, lmObs['sensorType'], lmObs['descriptor'])
                    self.graph.addNode(lmNode)
                else:
                    # update landmark guess as the average
                    lmPos = sensor.deadReckon(self.pos,lmObs['obs'])
                    lmNode.value = (lmNode.value + lmPos)/2.
                # create an edge between the nodes
                edge = ObservationEdge(sensor, lmObs['obs'], self.posNode, lmNode, "observation")
                self.graph.addEdge(edge)


    def reset(self):
        # initial position
        node = self.graph.nodes[0]
        # prior
        edge = self.graph.edges[0]

        self.posNode = node
        self.pos = node.value
        self.graph = Graph()
        self.graph.addNode(node)
        self.graph.addEdge(edge)


    def trajectory(self):
        positions = self.graph.getNodesOfType("position")
        sortedPos = sorted(positions, key=lambda x: x.descriptor)
        traj = map(lambda x: x.value, sortedPos)
        return array(traj)

    def trajectoryXY(self):
        traj = self.trajectory()
        return traj[:, 0:2]  # hack off the angle

    def position(self):
        traj = self.trajectory()
        pos = traj[-1]
        return pos

    def positionXY(self):
        traj = self.trajectory()
        pos = traj[-1]
        return pos[0:2]

#     def sensorOfType(self, s):
#         for sensor in self.sensors:
#             if sensor.type == s:
#                 return sensor

#     def wrap(self):
#         positions = np.array(pluck(self.state, "pos"))
#         landmarks = np.array(pluck(self.landmarks, "pos"))

#         posShape = positions.shape
#         posLen = positions.size
#         lmShape = landmarks.shape
#         lmLen = landmarks.size

#         X = positions.reshape((1, posLen))[0].tolist() + \
#             landmarks.reshape((1, lmLen))[0].tolist()
#         return X, posShape, posLen, lmShape, lmLen

#     def unwrap(self, X, posShape, posLen, lmShape, lmLen):
#         poss = np.array(X[0:posLen]).reshape(posShape)
#         lms = np.array(X[posLen:]).reshape(lmShape)

#         for i in range(len(poss)):
#             self.state[i]["pos"] = poss[i]
#         for i in range(len(lms)):
#             self.landmarks[i]["pos"] = lms[i]

