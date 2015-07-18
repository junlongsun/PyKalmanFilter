
import numpy as np

from numpy import dot, zeros, array, arctan, sqrt, sin, cos, vstack, hstack
#from numpy import normal, 
#import numpy

version = '0.1'

###############################################
class Sensor:
    def __init__(self,name):
        self.name = name
    def detect(self):
        pass
###############################################

class NormalSensor(Sensor):
    def detect(self,object,H,R):
        self.measurementSize = H.shape[0]
        self.measurement = dot(H, object.position) + np.random.multivariate_normal(zeros(self.measurementSize),R).reshape([self.measurementSize,1])

class BiasSensor(NormalSensor):
    def __init__(self,name, position, bias):
        NormalSensor.__init__(self,name)
        self.position = position
        self.bias = bias
    def detect(self, object1, H, R):
        x2 = object1.position[0]
        y2 = object1.position[2] # needs inprove, add some index to state position
        x1 = self.position[0]
        y1 = self.position[1]
     
        theta = abs(arctan((x2-x1)/(y2-y1)))

        range1 = sqrt((x2-x1)**2+(y2-y1)**2)

        G = vstack([ hstack([sin(theta), range1*cos(theta)]) , hstack([cos(theta), -range1*sin(theta)]) ])

        biasM = dot(G, self.bias)    
        self.measurementSize = H.shape[0]
        self.measurement = dot(H, object1.position) + biasM + np.random.multivariate_normal(zeros(self.measurementSize),R).reshape([self.measurementSize,1])
