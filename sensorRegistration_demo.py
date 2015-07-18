
from numpy import diag, array, eye, zeros, arctan, sqrt, sin, cos, vstack, hstack

version = '0.1'

dt = 3.0

X = array([230.0,-0.2,500,0.1]).reshape([4,1])
P = diag([1.0,0.01,1.0,0.01])**2

E = zeros([4,4])

A = array([[1.0, dt, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, dt], [0.0, 0.0, 0.0, 1.0]])
Qvector = array([0.3,0.01,0.2,0.05])*0.02
Q = diag(Qvector)**2


H = array([[1, 0, 0, 0], [0, 0, 1, 0]], dtype=float)
Rvector1 = array([0.05,0.0056])*2
R1 = diag(Rvector1)
Rvector2 = array([0.05,0.0056])*2
R2 = diag(Rvector2)

N_iter =50

position1 = array([0,0]).reshape([2,1])
position2 = array([300,0]).reshape([2,1])

bias1 = array([10, 0.0505]).reshape([2,1])
#bias2 = array([8, 0.0387])
bias2 = array([8,0.0087]).reshape([2,1])

mu = zeros([4,1]).reshape([4,1])
sigma = diag([1, 0.01, 1, 0.01])**2

def getG(object1, sensor):
    x2 = object1.position[0]
    y2 = object1.position[2] # needs inprove, add some index to state position
    x1 = sensor.position[0]
    y1 = sensor.position[1]
    theta = abs(arctan((x2-x1)/(y2-y1)))
    range1 = sqrt((x2-x1)**2+(y2-y1)**2)
    G = vstack([ hstack([sin(theta), range1*cos(theta)]) , hstack([cos(theta), -range1*sin(theta)]) ])
    return G