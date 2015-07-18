#import numpy as np
#import scipy as sp
#import matplotlib as mpl
#import matplotlib.pyplot as plt

from Object import NoisedPlane
from Filter import Estimate, KalmanFilter, KalmanFilterDecentrailized, BiasEstimate, VBbiasFilter
from Sensor import BiasSensor
from Observer import KalmanrObserver, KalmanrObserverDecentralized, VBbiasObserverDecentralized

from numpy import zeros, arange, array, vstack, hstack, dot

import matplotlib.pyplot as plt

from sensorRegistration_demo import getG, X, P, A, Q, H, R1, R2, N_iter, position1, bias1, position2, bias2, mu, sigma, E, Rvector1, Rvector2

estimate = Estimate(X,P)
biasEstimate = BiasEstimate(X, P, mu)
biasEstimate.uncertainty2parameters(mu, sigma)

plane=NoisedPlane('plane',X)
plane.stateSize = A.shape[0]
# snesor 1
kf1=KalmanFilter('Kalman filter for sensor 1')
kf1.setPosition(estimate)
radar1=BiasSensor('radar 1', position1, bias1)
radar1.measurementSize = H.shape[0]
kfObserver1=KalmanrObserver('Kalmanr Observer for kf 1')
# snesor 2
kf2=KalmanFilter('Kalman filter for sensor 2')
kf2.setPosition(estimate)
radar2=BiasSensor('radar 2', position2, bias2)
radar2.measurementSize = H.shape[0]
kfObserver2=KalmanrObserver('Kalmanr Observer for kf 2')
#decentralized kf
kfdc=KalmanFilterDecentrailized('Kalman filter Centralized')
kfdc.setPosition(estimate)
kfObserverDC=KalmanrObserverDecentralized('Kalmanr Observer for center')
# allocate space for arrays
kfdc=KalmanFilterDecentrailized('Kalman filter Centralized')
kfdc.setPosition(estimate)
kfObserverDC=KalmanrObserverDecentralized('Kalmanr Observer for center')
# variational filtering 
vbf = VBbiasFilter('Variational Bayesian Filtering')
vbf.setPosition(biasEstimate)
vbObserverDC=VBbiasObserverDecentralized('VB bias Observer Decentralized')
# Applying the Kalman Filter
for k in arange(0, N_iter):
    ##############################
    #sensor 1
    kf1.predict(A, Q)
    #sensor 2
    kf2.predict(A,Q)
    #center
    kfdc.predict(A, Q)
    #vb
    vbf.calcUncertain()
    vbf.timeEvolution(1)
    vbf.predict(A, Q, E)
    ##############################
    plane.move(A, Q)
    ##############################
    #sensor 1
    radar1.detect(plane, H, R1)
    #sensor 2
    radar2.detect(plane, H, R2)    
    ##############################
    kfdc.release()
    #sensor 1
    kf1.update(radar1, H, R1)
    kfdc.collect(kf1, H, R1)
    #sensor 2
    kf2.update(radar2, H, R2)
    kfdc.collect(kf2, H, R2)
    kfdc.update()
    # vb
    measurement = vstack([radar1.measurement.reshape([2,1]), radar2.measurement.reshape([2,1])])
    H0 = vstack([H, H])
    G0 = vstack([ hstack([getG(plane,radar1), zeros([2,2])]), hstack([zeros([2,2]),getG(plane,radar2)]) ])
    R0part1 = dot(getG(plane,radar1), Rvector1).reshape([2,1])
    R0part2 = dot(getG(plane,radar2), Rvector2).reshape([2,1])
    R0 = vstack([ hstack([ dot(R0part1, R0part1.T), zeros([2,2]) ]), hstack([ zeros([2,2]), dot(R0part2, R0part2.T) ]) ])    
    vbf.update(measurement, H0, R0, G0)
    ##############################
    kfObserver1.recieve(kf1, plane, radar1, k)
    kfObserver2.recieve(kf2, plane, radar2, k)
    kfObserverDC.recieve(kfdc, plane, k)
    vbObserverDC.recieve(vbf, plane, k)
    ##############################


totalReal1 = kfObserver1.data['real']
totalState1 = kfObserver1.data['state']
totalMeasurement1 = kfObserver1.data['measurement']
plt.plot(totalReal1[:,0], totalReal1[:,2], 'g-')
plt.plot(totalState1[:,0], totalState1[:,2], 'y.-')
plt.plot(totalMeasurement1[:,0], totalMeasurement1[:,1], 'bo')

#hold('on')
totalState2 = kfObserver2.data['state']
totalMeasurement2 = kfObserver2.data['measurement']
plt.plot(totalState2[:,0], totalState2[:,2], 'm.-')
plt.plot(totalMeasurement2[:,0], totalMeasurement2[:,1], 'co')

totalStateDC = kfObserverDC.data['state']
plt.plot(totalStateDC[:,0], totalStateDC[:,2], 'k.-')

totalStateVB = vbObserverDC.data['state']
plt.plot(totalStateVB[:,0], totalStateVB[:,2], 'r.-')

#plt.savefig('Kalman.eps')
plt.xlabel('x')
plt.ylabel('y')
plt.legend(['Real state', 
            'Kalman filter (Sensor 1)', 
            'Measurements (Sensor 1)', 
            'Kalman filter (Sensor 2)', 
            'Measurements (Sensor 2)',
            'Kalman filter (Fusion Center - Decentralized)',
            'VB-filter (Fusion Center - Decentralized)'])
plt.title('Filter results')

plt.show()
