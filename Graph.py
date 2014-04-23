"""
This is meant to be a general graph optimization tool.

references:
http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf
http://people.csail.mit.edu/kaess/pub/Dellaert06ijrr.pdf

inputs:
  nodes
  edges
  edge potential measurement
  edge potential function
  edge potential function covariance


in the case of SAM:
  nodes are landmarks (x,y) and position (x,y,a)
  edges are motions or observations
  edge potential measurement is either the command (forward, turn) or observation (distance, angle)
  edge potential function computes the expected measurement given both nodes
  edge potential function covariance is gaussian error covariance expected
"""

from pylab import *
from pprint import pprint

# TODO
# mohalanobis distance
# theano jacobian
# SAM integration

nodeIdx = 0
nodes = []

edgeIdx = 0
edges = []


"""
robot:
    motion model:
        simMove - robot simulated motion
        motionModel
        jacobian
        mohalanobis
    sensors:
        sensor model
            simSense - simulated sensing with map
            observationModel
            jacobianLandmark
            jacobianPosition
            mohalanobis


    def move:
        generate a new node and an edge
        nextNode = Node(naivePosition)
        edge = Edge(cmd,motionCov,prevNode, nextNode, motionModel, etc. )

    def sense:
        generate new nodes if necessary and new edges

"""


class Node:

    def __init__(self, value, other={}):
        # SAM: value is a landmark (x, y) or a position (x, y, angle)
        self.value = value
        self.other = other
        # keep track of which node
        globals nodes, nodeIdx
        self.idx = nodeIdx
        nodeIdx = nodeIdx + 1
        nodes.append(self)


class Edge:

    def __init__(self,
                 value,
                 covariance,
                 node1,
                 node2,
                 potentialFunction,
                 potentialJacobian1,
                 potentialJacobian2,
                 predictionError):
        # SAM: value is a command (forward, turn)
        #      or an observation (distance, angle)
        self.value = value
        self.covariance = covariance
        # SAM: node1 is a position
        #      node2 is either another position or a landmark
        self.node1 = node1
        self.node2 = node2
        self.potentialFunction = potentialFunction
        self.potentialJacobian1 = potentialJacobian1
        self.potentialJacobian2 = potentialJacobian2
        self.predictionError = predictionError
        # keep track of which edge
        globals edges, edgeIdx
        self.idx = edgeIdx
        edgeIdx = edgeIdx + 1
        edges.append(self)

    def potential(self):
        # SAM: f(x_{t-1}, u_{t}) or h(x_{t},l_k)
        return self.potentialFunction(self.value, self.node1, self.node2)

    def predictionError(self):
        return self.predictionError(self.value, self.node1, self.node2)

    def jacobian1(self):
        # Jacobian of the potential with respect to node1
        return self.potentialJacobian1(self.value, self.node1, self.node2)

    def jacobian2(self):
        # Jacobian of the potential with respect to node2
        return self.potentialJacobian2(self.value, self.node1, self.node2)


def emptyList(shape):
    arr = zeros(shape)
    return arr.tolist()


def arrayToList(arr):
    if type(arr) == type(array([])):
        return arrayToList(arr.tolist())
    elif type(arr) == type([]):
        return [arrayToList(a) for a in arr]
    else:
        return arr


def prettyArray(arr):
    pprint(arrayToList(arr))

# We are supplied with a set of nodes and edges
# first we will fill this list with the appropriate jacobians and vectors
# then we will flatten into a sparse matrix
A = emptyList([len(edges), len(nodes)])
b = emptyList([len(edges)])

for edge in edges:
    idxEdge = edge.idx
    idxNode1 = edge.node1.idx
    idxNode2 = edge.node2.idx

    A[idxEdge][idxNode1] = edge.jacobian1()
    A[idxEdge][idxNode2] = edge.jacobian2()
    b[idxEdge] = edge.predictionError()


def flattenMatrix(A):
    """
    Given a sparse (or dense) list of matricies, this function
    will flatten this matrix into a 2D matrix filling in zeros 
    where no matrices are specified
    """
    rows, cols = array(A).shape

    rowSizes = [0] * rows
    colSizes = [0] * cols

    for i in range(len(A)):
        for j in range(len(A[i])):
            elem = A[i][j]
            if type(elem) == type([]) or type(elem) == type(array([])):
                r, c = array(elem).shape
                if rowSizes[i] != 0 and rowSizes[i] != r:
                    print "ERROR: inconsisten row sizes"
                else:
                    rowSizes[i] = r
                if colSizes[j] != 0 and colSizes[j] != c:
                    print "ERROR: inconsisten col sizes"
                else:
                    colSizes[j] = c

    rowIndexes = [0] + add.accumulate(rowSizes).tolist()
    colIndexes = [0] + add.accumulate(colSizes).tolist()

    B = zeros([sum(rowSizes), sum(colSizes)])
    for i in range(len(A)):
        for j in range(len(A[i])):
            rowStart = rowIndexes[i]
            rowEnd = rowIndexes[i + 1]
            colStart = colIndexes[j]
            colEnd = colIndexes[j + 1]
            B[rowStart:rowEnd, colStart:colEnd] = array(A[i][j])

    return B


A = flattenMatrix(A)
b = flattenMatrix(b)

pinv(A)
