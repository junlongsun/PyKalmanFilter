
import numpy as np

from numpy import dot, zeros, array, concatenate, shape, empty, vstack, append, dtype

version = '0.1'

###############################################
class Observer:
    def __init__(self, name):
        self.name = name
        self.totalStep =0
    def recieve(self, filter1):
        pass
    def result(self):
        pass
###############################################

class KalmanrObserver(Observer):
    def __init__(self, name):
        Observer.__init__(self, name)
        self.type = 'Kalman Filter'    
    def recieveTime(self, k):
        self.timeStep = k        
    def recieveFilter(self, filter1):        
        self.state = filter1.state
        self.stateSize = filter1.stateSize     
        self.covariance = filter1.covariance        
    def recieveObject(self, object1):
        self.real  = object1.position
    def recieveSensor(self, sensor):
        self.measurement  = sensor.measurement
        self.measurementSize  = sensor.measurementSize
    def setStructure(self):
#        print self.state.shape
#        print self.covariance.shape
#        print self.real.shape
#        print self.measurement.shape
        self.ObserverStrucutre = dtype([('time step', np.int, 16), 
                                            ('state', np.float64, (self.stateSize,1)),
                                            ('covariance', np.float64, (self.stateSize,self.stateSize)),
                                            ('real', np.float64, (self.stateSize,1)),
                                            ('measurement', np.float64, (self.measurementSize,1))])    
        self.obserVedvector = array([(self.timeStep, self.state, self.covariance, self.real, self.measurement)], dtype=self.ObserverStrucutre)
    def write(self):
        if self.timeStep == 0:            
            self.data = self.obserVedvector
        else:
            self.data = append(self.data, self.obserVedvector)
    def recieve(self, filter1, object1, sensor, k):   
        self.recieveTime(k)
        self.recieveFilter(filter1)
        self.recieveObject(object1)
        self.recieveSensor(sensor)
        self.setStructure()
        self.write()    
        
    def result(self):
        print ('Kalman Filter Results at time-step %d' % self.timeStep)
        
class KalmanrObserverDecentralized(KalmanrObserver):
    def __init__(self, name):
        KalmanrObserver.__init__(self, name)
        self.type = 'Kalmanr Observer Decentralized'
        
    def setStructure(self):
        self.ObserverStrucutre = dtype([('time step', np.int, 16), 
                                            ('state', np.float64, (self.stateSize,1)),
                                            ('covariance', np.float64, (self.stateSize,self.stateSize)),
                                            ('real', np.float64, (self.stateSize,1))])

        self.obserVedvector = array([(self.timeStep, self.state, self.covariance, self.real)], dtype=self.ObserverStrucutre)
    
    def recieve(self, filter1, object1, k):   
        self.recieveTime(k)
        self.recieveFilter(filter1)
        self.recieveObject(object1)    
        self.setStructure()
        self.write()

class VBbiasObserverDecentralized(KalmanrObserverDecentralized):
    def __init__(self, name):
        KalmanrObserver.__init__(self, name)
        self.type = 'VB bias Observer Decentralized'
    def recieveFilter(self, filter1):
        KalmanrObserverDecentralized.recieveFilter(self, filter1)
        self.uncertain = filter1.uncertain
        self.uncertainSize = filter1.uncertainSize
    def setStructure(self):
        self.ObserverStrucutre = dtype([('time step', np.int, 16), 
                                            ('state', np.float64, (self.stateSize,1)),
                                            ('covariance', np.float64, (self.stateSize,self.stateSize)),
                                            ('real', np.float64, (self.stateSize,1)),
                                            ('uncertain', np.float64, (self.uncertainSize,1))])
        self.obserVedvector = array([(self.timeStep, self.state, self.covariance, self.real, self.uncertain)], dtype=self.ObserverStrucutre)
                             