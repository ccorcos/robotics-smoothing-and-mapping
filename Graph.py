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



could do factors for edges with n nodes

"""

from pylab import *
from utils import *
from scipy.sparse.linalg import spsolve


class Node:

    def __init__(self, value, nodeType, descriptor):
        # SAM: value is a landmark (x, y) or a position (x, y, angle)
        self.value = value
        self.nodeType = nodeType
        self.descriptor = descriptor
        # the Graph will set an index to keep track of where it is
        # self.idx = idx


class MotionEdge:

    def __init__(self, model, value, node1, node2, edgeType):
        self.model = model
        self.value = value  # cmd
        self.node1 = node1
        self.node2 = node2
        self.edgeType = edgeType

    def error(self):
        """The error associated with this edge: f(x_0, u_1) - x_1"""
        pos0 = self.node1.value
        pos1 = self.node2.value
        cmd = self.value
        err = self.model.move(pos0, cmd) - pos1
        err[2] = wrapAngle(err[2])
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
        G = -eye(len(pos0))
        err = self.error()

        # transform via covariance so we can do the L2 distance
        # as opposed to computing the mohalanobis distance
        C = self.model.covariance(pos0, cmd)
        Fm = mohalanobis(C, F)
        Gm = mohalanobis(C, G)
        a = mohalanobis(C, -err)

        return Fm, Gm, a


class ObservationEdge:

    def __init__(self, model, value, node1, node2, edgeType):
        self.model = model
        self.value = value  # obs
        self.node1 = node1
        self.node2 = node2
        self.edgeType = edgeType

    def error(self):
        """The error associated with this edge: h(x, l) - z"""
        pos = self.node1.value
        lm = self.node2.value
        obs = self.value
        err = self.model.sense(pos, lm) - obs
        err[1] = wrapAngle(err[1])
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
        Hm = mohalanobis(C, H)
        Jm = mohalanobis(C, J)
        c = mohalanobis(C, -err)
        return Hm, Jm, c

class PriorEdge:

    def __init__(self, value, node, covariance):
        self.value = value
        self.covariance = covariance
        self.node1 = node
        self.node2 = node
        self.edgeType = "prior"

    def error(self):
        """The error associated with this edge: f(x_0, u_1) - x_1"""
        err = array(self.node1.value) - array(self.value)
        err[2] = wrapAngle(err[2])
        return err

    def linearized(self):
        # transform via covariance so we can do the L2 distance
        # as opposed to computing the mohalanobis distance
        C = self.covariance
        J = mohalanobis(C,-eye(len(self.value)))
        return J,J,-self.error()

class Graph:

    def __init__(self):
        self.nodeIdx = 0
        self.edgeIdx = 0
        self.nodes = []
        self.edges = []

    def addNode(self, node):
        node.idx = self.nodeIdx
        self.nodeIdx = self.nodeIdx + 1
        self.nodes.append(node)
        # pr(2,"adding node")
        # pr(3,"type", node.nodeType)
        # pr(3,"value", node.value)
        # pr(3,"idx", node.idx)

    def addEdge(self, edge):
        edge.idx = self.edgeIdx
        self.edgeIdx = self.edgeIdx + 1
        self.edges.append(edge)
        # pr(2,"adding edge")
        # pr(3,"type", edge.edgeType)
        # pr(3,"value", edge.value)
        # pr(3,"idx", edge.idx)
        # pr(3, 'error', edge.error())
        # pr(4, "from value", edge.node1.value)
        # pr(4, 'from idx', edge.node1.idx)
        # pr(4, "to value", edge.node2.value)
        # pr(4, 'to idx', edge.node2.idx)
        

    def getNodesOfType(self, nodeType):
        return filter(lambda x: x.nodeType == nodeType, self.nodes)

    def getEdgesOfType(self, edgeType):
        return filter(lambda x: x.edgeType == edgeType, self.edges)

    def getNodeOfTypeAndDescriptor(self, nodeType, descriptor):
        n = filter(
            lambda x: x.nodeType == nodeType and x.descriptor == descriptor, self.nodes)
        if len(n) > 1:
            ex("ERROR: multiple nodes of the same descriptor!")
        if len(n) == 0:
            return None
        else:
            return n[0]

    def optimizationStep(self):
        """
        Optimize the graph. Nodes have a value. Edges have a value, and an error
        function with respect to that value and the node values. 

        This optimization assumes the edge values are assumed and the node values
        are to be optimized. This is done by linearizing the edge error functions
        and solving a sparse least squares problem. 

        This is considered one iteration of a nonlinear graph optimization.
        """
        # first lets form the structure, Ax=b
        # each element is a jacobian or a vector
        # later we will flatten into a 2D matrix
        A = emptyList([len(self.edges), len(self.nodes)])
        b = emptyList([len(self.edges)])

        # print the sparsity
        S = emptyList([len(self.edges), len(self.nodes)])

        for edge in self.edges:
            # print edge.idx
            # print edge.edgeType
            # print edge.node1.idx
            # print edge.node2.idx
            
            idxEdge = edge.idx
            idxNode1 = edge.node1.idx
            idxNode2 = edge.node2.idx

            J1, J2, bi = edge.linearized()


            # print J1
            # print J2
            # print bi
            # wait()
            A[idxEdge][idxNode1] = J1
            A[idxEdge][idxNode2] = J2
            b[idxEdge] = bi

            S[idxEdge][idxNode1] = 1
            S[idxEdge][idxNode2] = 1
            # print A
            # print S
            # print b 
            # wait()
            
            # pr(2, "linearizing edge")
            # pr(3,"type", edge.edgeType)
            # pr(3,"value", edge.value)
            # pr(3,"idx", edge.idx)
            # pr(3, 'error', edge.error())
            # pr(4, "from value", edge.node1.value)
            # pr(4, 'from idx', edge.node1.idx)
            # pr(4, "to value", edge.node2.value)
            # pr(4, 'to idx', edge.node2.idx)
            # pr(4, "J1", J1)
            # pr(4, "J2", J2)
            # pr(4, "bi", bi)
            # wait()

        A = flattenMatrix(A)
        b = flattenVector(b)

        # print A.shape
        # print b.shape

        dx = inner(pinv(A),b)

        # pr(2, "b", b)
        # pr(2, "dx", dx)
        # pr(2, A)
        # wait()

        # print A
        # print b
        # print dx
        # wait()

        # Now update the graph values
        nodes = sorted(self.nodes, key=lambda x: x.idx)
        # print map(lambda x: x.idx, nodes)
        # wait()
        startIdx = 0
        # Add the dx to each node
        for i in range(len(nodes)):
            node = nodes[i]
            if node.idx != i:
                ex("ERROR: wrong node!")
            dxi = dx[startIdx:startIdx+len(node.value)]
            pr(2, "update node")
            pr(3, "type", node.nodeType)
            pr(3, "idx", node.idx)
            pr(3, "value", node.value)
            pr(3, "dx", dxi)
            node.value = node.value + dxi*0.5
            startIdx = startIdx + len(node.value)



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
                    print "ERROR: inconsistent row sizes"
                else:
                    rowSizes[i] = r
                if colSizes[j] != 0 and colSizes[j] != c:
                    print "ERROR: inconsistent col sizes"
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

def flattenVector(A):
    """
    """

    rows = len(A)

    rowSizes = [0] * rows
    for i in range(len(A)):
        elem = A[i]
        if type(elem) == type([]) or type(elem) == type(array([])):
            r = len(elem)
            if rowSizes[i] != 0 and rowSizes[i] != r:
                print "ERROR: inconsistent row sizes"
            else:
                rowSizes[i] = r

    rowIndexes = [0] + add.accumulate(rowSizes).tolist()

    B = zeros([sum(rowSizes)])
    for i in range(len(A)):
        rowStart = rowIndexes[i]
        rowEnd = rowIndexes[i + 1]
        B[rowStart:rowEnd] = array(A[i])

    return B


def mohalanobis(covariance, M):
    # inner(a,b) = a*b'
    return inner(pinv(sqrt(covariance)).T, M.T)
    # return M