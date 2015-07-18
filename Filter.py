

from numpy import dot, array, zeros, vstack, hstack, diag
from numpy.linalg import inv

version = '0.1'

###############################################
class Filter:
    def __init__(self,name):
        self.name = name
    def setPosition(self):
        pass
    def predict(self):
        pass
    def update(self):
        pass
###############################################

class Estimate:
    def __init__(self, state, covariance, timeStep = 0):
        self.state = state
        self.covariance = covariance
        self.timeStep = timeStep
    def update(self, estimate):
        self.estimate = estimate
        self.timeStep += 1 

class KalmanFilter(Filter):
    def __init__(self,name):
        Filter.__init__(self,name)
        self.type = 'Kalman'
        self.state = []
        self.covariance = []
        print '(Initialized: %s)' %self.type
    def setPosition(self, estimate):
        self.state = estimate.state
        self.covariance = estimate.covariance
    def predict(self, A, Q):
        self.stateSize = A.shape[0]
        self.state = dot(A, self.state)
        self.covariance = dot(A, dot(self.covariance, A.T)) + Q
        self.statePre = self.state
        self.covariancePre = self.covariance  
    def update(self, radar, H, R):
        IM = dot(H, self.state)
        IS = R + dot(H, dot(self.covariance, H.T))
        K = dot(self.covariance, dot(H.T, inv(IS)))
        self.state = self.state + dot(K, (radar.measurement-IM))
        self.covariance = self.covariance - dot(K, dot(IS, K.T))
        
class KalmanFilterDecentrailized(KalmanFilter):
     def __init__(self,name):
         KalmanFilter.__init__(self,name)
         self.type = 'Decretrailized Kalman'
         self.collectNum = 0
         print '(Extended: %s)' %self.type    
     def release(self):
         self.collectNum = 0       
         
     def collect(self, filter1, H, R):
         if self.collectNum == 0:
             #self.sizeBar = []
             #self.hBar = []
             #self.rBar = []
             self.statePreBar = []
             self.covariancePreBar = []  
             self.stateBar = []
             self.covarianceBar = []         
         #self.sizeBar += [sensor.measurementSize]
         #self.hBar += [H]
         #self.rBar += [R]
         self.statePreBar += [filter1.statePre]
         self.covariancePreBar += [filter1.covariancePre]  
         self.stateBar += [filter1.state]
         self.covarianceBar += [filter1.covariance]             
         self.collectNum += 1   
         
     def update(self):
        invP = zeros([self.stateSize,self.stateSize])
        invPX = zeros([self.stateSize,]).reshape(self.stateSize,1)
        for i in range(0, self.collectNum):
            tempPinv = inv(self.covarianceBar[i])
            tempPinvPre = inv(self.covariancePreBar[i])
            invP = invP + tempPinv - tempPinvPre         
            invPX = invPX + dot(tempPinv,self.stateBar[i]) - dot(tempPinvPre,self.statePreBar[i])
        
        invP0 = inv(self.covariance)
        tempP = invP0 + invP
        self.covariance = inv(tempP)
        tempX = dot(invP0, self.state) + invPX
        self.state = dot(self.covariance, tempX)       

class BiasEstimate(Estimate):
    def __init__(self, state, covariance, uncertain, timeStep = 0):
        self.state = state
        self.covariance = covariance
        self.timeStep = timeStep
        self.uncertain = uncertain
    def uncertainty1parameters(self, para1):
        self.para1 = para1
    def uncertainty2parameters(self, para1, para2):
        self.uncertainty1parameters(para1)
        self.para2 = para2
    def update(self, estimate):
        self.estimate = estimate
        self.timeStep += 1 

class VBFilter(Filter):
    def __init__(self,name):
        Filter.__init__(self,name)
        self.type = 'VB Filter'
        self.state = []
        self.covariance = []
        self.uncertain = []
        print '(Initialized: %s)' %self.type
    def timeEvolution(self):
         pass

class VBbiasFilter(VBFilter, KalmanFilter):
     def __init__(self, name):
         VBFilter.__init__(self, name)
         self.type = 'Variational Bayesian Filtering for bias sensors'
         print '(Extended: %s)' %self.type
     def setPosition(self, biasEstimate):
         KalmanFilter.setPosition(self, biasEstimate)
         self.uncertainIndex = 2
         self.mu = biasEstimate.para1
         self.sigma = biasEstimate.para2
     def calcUncertain(self):
         self.uncertain = self.mu
         self.uncertainSize = self.uncertain.shape[0]
     def predict(self, A, Q, E):
         KalmanFilter.predict(self, A, Q)
         self.state = self.state + dot(E, self.uncertain)
         self.uncertainSize = E.shape[0]
     def timeEvolution(self, rho):
         self.mu = self.mu
         self.sigma = self.sigma * rho
     def update(self, measurement, H, R, G):
         R = diag(diag(R))
         zerosBlock = zeros([self.stateSize, self.uncertainSize])
         MatrixB = vstack([H.T, G.T])
         MatrixD = inv(R)

         MatrixC = hstack([H, G])
         MatrixA = vstack([hstack([inv(self.covariance), zerosBlock]),hstack([zerosBlock.T,inv(self.sigma)])])
         temp1 = inv( inv(MatrixD) + dot(MatrixC, dot(inv(MatrixA), MatrixB)) )
         MatrixInv = inv(MatrixA) - dot(inv(MatrixA), dot(MatrixB, dot(temp1, dot( MatrixC , inv(MatrixA)))))
         B = vstack([ (dot(inv(self.covariance), self.state) + dot(H.T, dot(inv(R), measurement))) , (dot(inv(self.sigma),self.mu) + dot(G.T, dot(inv(R), measurement)))])
         X = dot(MatrixInv,B) #(8,4)
         self.state = X[:self.stateSize] #(4,4)
         self.mu = X[self.stateSize:]
         self.covariance = inv(inv(self.covariance) + dot(H.T,dot(inv(R),H)))
         self.sigma = inv(inv(self.sigma) + dot(G.T,dot(inv(R),G)))

