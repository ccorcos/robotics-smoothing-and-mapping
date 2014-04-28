import theano

# def motionModel(pos, cmd):
#     # f(x_{t-1},u_{t})
#     # the motion model
#     x = pos[0]
#     y = pos[1]
#     a = pos[2]
#     forward = cmd[0]
#     turn = cmd[1]
#     a = a + turn
#     x = x + theano.tensor.cos(a) * forward
#     y = y + theano.tensor.sin(a) * forward
#     return [x, y, a]

# pos = theano.tensor.vector("position")
# cmd = theano.tensor.vector("command")
# out = motionModel(pos, cmd)
# j0 = theano.tensor.grad(out[0],[pos])
# j1 = theano.tensor.grad(out[1],[pos])
# j2 = theano.tensor.grad(out[2],[pos])
# move = theano.function([pos, cmd], out)
# J = theano.function([pos,cmd], theano.tensor.stack(j0[0],j1[0],j2[0]))
# print move([0, 0, 0], [1, 0.1])
# print J([0,0,0],[1,0.1])
# print J

from pylab import *

def sense(pos, landmark):
    """The observation model for this sensor, h(x,l)"""

    x = pos[0]
    y = pos[1]
    a = pos[2]
    xy = np.array([x, y])

    d = norm(landmark - xy)  # distance to the landmark

    c = np.array(landmark - xy)
    b = np.array([cos(a), sin(a)])

    # sign(sintheta)*acos(costheta)
    t = -sign(cross(c, b) / d) * arccos(dot(c, b) / d)

    return array([d, t])

print sense([0,0,0],[1,1])

pos = theano.tensor.vector("position")
lm = theano.tensor.vector("landmark")
out = sense(pos, lm)
# j0 = theano.tensor.grad(out[0],[pos])
# j1 = theano.tensor.grad(out[1],[pos])
# obs = theano.function([pos, lm], out)
# J = theano.function([pos,lm], theano.tensor.stack(j0[0],j1[0],j2[0]))
# print obs([0, 0, 0], [1, 0.1])
# print J([0,0,0],[1,0.1])
# print J