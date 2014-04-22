import theano
from pylab import *

u = []  # motion commands
z = []  # observations

x = []  # robot positions
l = []  # landmark positions

# f(x,u) = motion model
# h(x,l) = observation model

# each sensor, define a covariance and observation function with theano
# for each motion model, define a covarance and motion function with theano

# theano trajectory cost function
# theano observation cost function


import theano

#
#
#
# L2 Norm
x = theano.tensor.dvector('x')
y = theano.tensor.dvector('y')
p = theano.tensor.dscalar('p')
pnorm = theano.function([x, y, p], pow(theano.tensor.sum((x - y) ** p), 1 / p))
print pnorm([0, 0], [1, 1], 2)

L2norm = theano.function([x, y], pnorm(x, y, 2))
print L2norm([0, 0], [1, 1])


#
#
#
# observation model
posx = theano.tensor.dscalar('posx')  # robot x position
posy = theano.tensor.dscalar('posy')  # robot y position
posa = theano.tensor.dscalar('posa')  # robot angle position

lmx = theano.tensor.dscalar('lmx')  # landmark x position
lmy = theano.tensor.dscalar('lmy')  # landmark y position

pos = [posx, posy]
lm = [lmx, lmy]

dist = pnorm([pos, lm, 2])

diff = lm - pos
head = [cos(posa), sin(posa)]
angle = -sign(cross(diff, head) / dist) * arccos(dot(diff, head) / dist)

obs = theano.function([posx, posy, posa], [dist, angle])

#
#
#
# motion model

#
#
#
# build graph model


#
#
#
# cost function


#
#
#
# train iteration function
