from numpy import diag, array

version = '0.1'

X = array([-0.50, 0.0, 2.0, -2.0]).reshape([4,1])
P = diag((0.1, 0.1, 0.1, 0.1))

A = array([[1.0000, 0.0, 0.1000, 0.0], [0.0, 1.0000, 0.0, 0.1000], [0.0, 0.0, 1.0000, 0.0], [0.0, 0.0, 0.0, 1.0000]])
Q = array([[0.0, 0.0, 0.0005, 0.0], [0, 0.0, 0.0, 0.0005], [0.0005, 0.0, 0.01, 0.0],[0.0, 0.0005, 0.0, 0.01]])

H = array([[1.0, 0.0, 0.0, 0.0], [0, 1.0, 0.0, 0.0]])
R = array([[0.05, 0.0], [0.0, 0.05]])

N_iter = 50


