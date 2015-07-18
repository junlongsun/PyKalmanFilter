#import numpy as np
#import scipy as sp
#import matplotlib as mpl
#import matplotlib.pyplot as plt

from Object import Plane
from Filter import Estimate, KalmanFilter, KalmanFilterDecentrailized
from Sensor import NormalSensor
from Observer import KalmanrObserver, KalmanrObserverDecentralized

from numpy import zeros, arange, array

import matplotlib.pyplot as plt

from cluster_demo import X, P, A, Q, H, R, N_iter

estimate=Estimate(X,P)

plane=Plane('plane',X)
plane.stateSize = A.shape[0]
# snesor 1
kf1=KalmanFilter('Kalman filter for sensor 1')
kf1.setPosition(estimate)
radar1=NormalSensor('radar 1')
radar1.measurementSize = H.shape[0]
kfObserver1=KalmanrObserver('Kalmanr Observer for kf 1')
# snesor 2
kf2=KalmanFilter('Kalman filter for sensor 2')
kf2.setPosition(estimate)
radar2=NormalSensor('radar 2')
radar2.measurementSize = H.shape[0]
kfObserver2=KalmanrObserver('Kalmanr Observer for kf 2')
#decentralized kf
kfdc=KalmanFilterDecentrailized('Kalman filter Centralized')
kfdc.setPosition(estimate)
kfObserverDC=KalmanrObserverDecentralized('Kalmanr Observer for center')
# allocate space for arrays

# Applying the Kalman Filter
for k in arange(0, N_iter):
    ##############################
    #sensor 1
    kf1.predict(A,Q)
    #sensor 2
    kf2.predict(A,Q)
    #center
    kfdc.predict(A,Q)
    ##############################
    plane.move(A)
    ##############################
    #sensor 1
    radar1.detect(plane,H,R)
    #sensor 2
    radar2.detect(plane,H,R)    
    ##############################
    kfdc.release()
    #sensor 1
    kf1.update(radar1,H,R)
    kfdc.collect(kf1,H,R)
    #sensor 2
    kf2.update(radar2,H,R)
    kfdc.collect(kf2,H,R)
    kfdc.update()
    ##############################
    kfObserver1.recieve(kf1, plane, radar1, k)
    kfObserver2.recieve(kf2, plane, radar2, k)
    kfObserverDC.recieve(kfdc, plane, k)
    ##############################
    #reporter.result()
    
    #decentralized kf

#import pylab 
#print (Real[0,:])    
#print (Real[1,:])    
#pylab.figure()
#pylab.plot(kfObserver.totalReal[:,0], kfObserver.totalReal[:,1])
#pylab.plot(kfObserver.totalState[:,0], kfObserver.totalState[:,1])
#pylab.plot(kfObserver.measurement[0,0:-1], kfObserver.measurement[1,0:-1])
#pylab.show()

totalReal1 = kfObserver1.data['real']
totalState1 = kfObserver1.data['state']
totalMeasurement1 = kfObserver1.data['measurement']
plt.plot(totalReal1[:,0], totalReal1[:,1], 'g-')
plt.plot(totalState1[:,0], totalState1[:,1], 'r.-')
plt.plot(totalMeasurement1[:,0], totalMeasurement1[:,1], 'bo')

#hold('on')
totalState2 = kfObserver2.data['state']
totalMeasurement2 = kfObserver2.data['measurement']
plt.plot(totalState2[:,0], totalState2[:,1], 'm.-')
plt.plot(totalMeasurement2[:,0], totalMeasurement2[:,1], 'co')

totalStateDC = kfObserverDC.data['state']
plt.plot(totalStateDC[:,0], totalStateDC[:,1], 'k.-')

#plt.savefig('Kalman.eps')
plt.xlabel('x')
plt.ylabel('y')
plt.legend(['Real state', 
            'Kalman filter (Sensor 1)', 
            'Measurements (Sensor 1)', 
            'Kalman filter (Sensor 2)', 
            'Measurements (Sensor 2)',
            'Kalman filter (Fusion Center - Decentralized)'])
plt.title('Filter results')

plt.show()


    

