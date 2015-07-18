
from numpy import diag, array, eye

version = '0.1'

dt = 0.1

X = array([1.0, 1.0, 0.1, 0.1]).reshape([4,1])
P = diag((0.01, 0.01, 0.01, 0.01))

A = array([[1, 0, dt , 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
Q = eye(X.shape[0])*0.0001

H = array([[1, 0, 0, 0], [0, 1, 0, 0]])
R = eye(H.shape[0])*0.0001

N_iter = 40


