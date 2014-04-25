"""
This is meant to be a general graph optimization tool.

references:
http://ais.informatik.uni-freiburg.de/publications/papers/kuemmerle11icra.pdf
http://people.csail.mit.edu/kaess/pub/Dellaert06ijrr.pdf
# http://en.wikipedia.org/wiki/Linearization#Linearisation_of_a_multivariable_function



custom edge classes are needed for different types of observations
an edge class must have the following instance variables and methods:

self.value = the value observed. (SAM: command or sensor reading)
self.node1 = this is an undirected graph, but the order here matters
self.node2
def error(self) = the error between value and the potential function for 
    computing the value between the two nodes
def linearized(self) return 3 values
    the jacobian of error with respect to node1
    the jacobian of error with respect to node2
    the negative error

"""

from pylab import *
from utils import *

class Node:

    def __init__(self, value, nodeType, descriptor):
        # SAM: value is a landmark (x, y) or a position (x, y, angle)
        self.value = value
        self.nodeType = nodeType
        self.descriptor = descriptor
        # the Graph will set an index to keep track of where it is
        # self.idx = idx

class MotionEdge:
    def __init__(self, model, value, node1, node2):
        self.model = model
        self.value = value # cmd
        self.node1 = node1
        self.node2 = node2

    def error(self):
        """The error associated with this edge: f(x_0, u_1) - x_1"""
        pos0 = self.node1.value
        pos1 = self.node2.value
        cmd = self.value
        err = self.model.move(pos0, cmd) - pos1
        return err

    def linearized(self):
        """Solve by linearizing.
        http://en.wikipedia.org/wiki/Linearization#Linearisation_of_a_multivariable_function
        In general:
        error(node1, node2, values) + derror/dnode1 * dnode1 + derror/dnode2 * dnode2 = 0
        Therefor, we solve for dnode1 and dnode2
        derror/dnode1 * dnode1 + derror/dnode2 * dnode2 = -error(node1, node2, values)
        """
        pos0 = self.node1.value
        pos1 = self.node2.value
        cmd = self.value

        F = self.model.jacobianPosition(pos0, cmd)
        G = eye(len(pos0))
        err = self.error()

        # transform via covariance so we can do the L2 distance
        # as opposed to computing the mohalanobis distance
        C = self.model.covariance()
        Fm = mohalanobis(C,F)
        Gm = mohalanobis(C,G)
        a = mohalanobis(C,-err)

        return Fm,Gm,a

class ObservationEdge:
    def __init__(self, model, value, node1, node2):
        self.model = model
        self.value = value # obs
        self.node1 = node1
        self.node2 = node2

    def error(self):
        """The error associated with this edge: h(x, l) - z"""
        pos = self.node1.value
        lm = self.node2.value
        obs = self.value
        err = self.model.sense(pos, lm) - obs
        return err

    def linearized(self):
        """
        Solve by linearizing.
        http://en.wikipedia.org/wiki/Linearization#Linearisation_of_a_multivariable_function
        In general:
        error(node1, node2, values) + derror/dnode1 * dnode1 + derror/dnode2 * dnode2 = 0
        Therefor, we solve for dnode1 and dnode2
        derror/dnode1 * dnode1 + derror/dnode2 * dnode2 = -error(node1, node2, values)
        """
        pos = self.node1.value
        lm = self.node2.value
        obs = self.value

        H = self.model.jacobianPosition(pos, lm)
        J = self.model.jacobianLandmark(pos, lm)
        err = self.error()

        # transform via covariance so we can do the L2 distance
        # as opposed to computing the mohalanobis distance
        C = self.model.covariance()
        Hm = mohalanobis(C,H)
        Jm = mohalanobis(C,J)
        c = mohalanobis(C,-err)
        return F,G,a

class Graph:

    def __init__(self):
        # self.nodeIdx = 0
        # self.edgeIdx = 0
        self.nodes = []
        self.edges = []

    def addNode(self, node):
        self.nodes.append(node)

    def addEdge(self, edge):
        self.edges.append(edge)

    def getNodesOfType(self, nodeType):
        return filter(lambda x: x.nodeType == nodeType, self.nodes)

    def getNodeOfTypeAndDescriptor(self, nodeType, descriptor):
        n = filter(lambda x: x.nodeType == nodeType and x.descriptor == descriptor, self.nodes)
        if len(n) > 1:
            ex("ERROR: multiple nodes of the same descriptor!")
        if len(n) == 0:
            return None
        else:
            return n[0]

# def emptyList(shape):
#     arr = zeros(shape)
#     return arr.tolist()

# def arrayToList(arr):
#     if type(arr) == type(array([])):
#         return arrayToList(arr.tolist())
#     elif type(arr) == type([]):
#         return [arrayToList(a) for a in arr]
#     else:
#         return arr

# # We are supplied with a set of nodes and edges
# # first we will fill this list with the appropriate jacobians and vectors
# # then we will flatten into a sparse matrix
# A = emptyList([len(edges), len(nodes)])
# b = emptyList([len(edges)])

# for edge in edges:
#     idxEdge = edge.idx
#     idxNode1 = edge.node1.idx
#     idxNode2 = edge.node2.idx

#     A[idxEdge][idxNode1] = edge.jacobian1()
#     A[idxEdge][idxNode2] = edge.jacobian2()
#     b[idxEdge] = edge.predictionError()


# def flattenMatrix(A):
#     """
#     Given a sparse (or dense) list of matricies, this function
#     will flatten this matrix into a 2D matrix filling in zeros 
#     where no matrices are specified
#     """
#     rows, cols = array(A).shape

#     rowSizes = [0] * rows
#     colSizes = [0] * cols

#     for i in range(len(A)):
#         for j in range(len(A[i])):
#             elem = A[i][j]
#             if type(elem) == type([]) or type(elem) == type(array([])):
#                 r, c = array(elem).shape
#                 if rowSizes[i] != 0 and rowSizes[i] != r:
#                     print "ERROR: inconsisten row sizes"
#                 else:
#                     rowSizes[i] = r
#                 if colSizes[j] != 0 and colSizes[j] != c:
#                     print "ERROR: inconsisten col sizes"
#                 else:
#                     colSizes[j] = c

#     rowIndexes = [0] + add.accumulate(rowSizes).tolist()
#     colIndexes = [0] + add.accumulate(colSizes).tolist()

#     B = zeros([sum(rowSizes), sum(colSizes)])
#     for i in range(len(A)):
#         for j in range(len(A[i])):
#             rowStart = rowIndexes[i]
#             rowEnd = rowIndexes[i + 1]
#             colStart = colIndexes[j]
#             colEnd = colIndexes[j + 1]
#             B[rowStart:rowEnd, colStart:colEnd] = array(A[i][j])

#     return B


# A = flattenMatrix(A)
# b = flattenMatrix(b)

# # colamd, QR factorization
# d = pinv(A)*b

# where = 0
# # now, we go backwards. we add the correct element of d to all nodes and edges
# for i in range(len(nodes)):
#     node = nodes[i]
#     if node.idx != i:
#         ex("ERROR: wrong node!")
#     di = d[where:where+len(node.value)]
#     node.value = node.value + di
#     where = where + len(node.value)

