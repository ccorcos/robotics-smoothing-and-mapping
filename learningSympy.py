from pylab import *
from sympy import *
from sympy import sin, cos
# solve for analytical jacobians using sympy :)

def jacobian(arr, var):
    J = []
    for i in range(len(arr)):
        ji = []
        for j in range(len(var)):
            ji.append(arr[i].diff(var[j]))
        J.append(ji)
    return J


# # Motion Model
# x = Symbol("x", real=True)
# y = Symbol("y", real=True)
# a = Symbol("a", real=True)
# f = Symbol("f", real=True)
# t = Symbol("t", real=True)
# x1 = x + cos(a) * f
# y1 = y + sin(a) * f
# a1 = a + t
# print jacobian([x1,y1,a1],[x,y,a])


# Observation Model
x = Symbol("x", real=True)
y = Symbol("y", real=True)
a = Symbol("a", real=True)
w = Symbol("w", real=True)
z = Symbol("z", real=True)
xy = Matrix([x,y])
lm = Matrix([w,z])
c = lm-xy
d = c.norm()
t = atan2(c[1],c[0]) - a
print t
print jacobian([d,t],[x,y,a])
print jacobian([d,t],[w,z])
