#import numpy as np
#import scipy as sp
#import matplotlib as mpl
#import matplotlib.pyplot as plt

from Object import Plane
from Filter import Estimate, KalmanFilter
from Sensor import NormalSensor
from Observer import KalmanrObserver

from numpy import zeros, arange, array

import matplotlib.pyplot as plt

from cluster_demo import X, P, A, Q, H, R, N_iter

estimate=Estimate(X,P)

plane=Plane('plane',X)
plane.stateSize = A.shape[0]
kf=KalmanFilter('kalmanfilter')
kf.setPosition(estimate)
radar=NormalSensor('radar')
radar.measurementSize = H.shape[0]
kfObserver=KalmanrObserver('Kalmanr Observer')

# allocate space for arrays

# Applying the Kalman Filter
for k in arange(0, N_iter):
    kf.predict(A,Q)
    plane.move(A)
    radar.detect(plane,H,R)
    kf.update(radar,H,R)
    kfObserver.recieve(kf, plane, radar, k)
    #reporter.result()
  

#import pylab 
#print (Real[0,:])    
#print (Real[1,:])    
#pylab.figure()
#pylab.plot(kfObserver.totalReal[:,0], kfObserver.totalReal[:,1])
#pylab.plot(kfObserver.totalState[:,0], kfObserver.totalState[:,1])
#pylab.plot(kfObserver.measurement[0,0:-1], kfObserver.measurement[1,0:-1])
#pylab.show()

totalReal = kfObserver.data['real']
totalState = kfObserver.data['state']
totalMeasurement = kfObserver.data['measurement']
plt.plot(totalReal[:,0], totalReal[:,1], 'g-')
plt.plot(totalState[:,0], totalState[:,1], 'r.-')
plt.plot(totalMeasurement[:,0], totalMeasurement[:,1], 'bo')
plt.xlabel('x')
plt.ylabel('y')
plt.legend(['Real state', 'Kalman filter', 'Measurements'])
plt.title('Filter results')
#plt.savefig('Kalman.eps')
plt.show()


    

