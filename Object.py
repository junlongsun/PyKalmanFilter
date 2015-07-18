
version = '0.1'

from numpy import dot, zeros
import numpy as np

###############################################
class Object:
    def __init__(self,name):
        self.name = name
    def move(self):
        print 'I am moving'
###############################################

class Plane(Object):
    def __init__(self,name,position):
        Object.__init__(self,name)
        self.position = position
    def move(self,A):
        self.position = dot(A, self.position)
class NoisedPlane(Plane):
    def move(self, A, Q):
        self.position = dot(A, self.position) + np.random.multivariate_normal(zeros(A.shape[0]),Q).reshape([A.shape[0],1])
