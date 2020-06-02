#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Anwendung des zweidimenstionalen Kalman-Filters aus dem Juypter Notebook
# "Der mehrmensionale Kalman-Filter, Teil 2 Simulationen" auf das Sensorsignal
# des Ultraschallsensors SRF02 (Modus Messwerte in µs Laufzeit).
# Start: Abstand 2000 µs, Abstandsänderung 0 µs/s, Varianz Abstand 1000 µs,
# Varianz Abstandsänderung 1000 µs/s
# Sensor Standardabweichung: sigma_z = 5 µs,
# Standardabweichung Abstandsänderung Prozess: Wurzel(Q_var) = 3 µs/s.
# S. Mack, 2.6.20

import sys
import smbus
import time
import numpy as np
from scipy.linalg import inv

print('Python-Interpreter: {}\n'.format(sys.version))
i2c = smbus.SMBus(2)
print('i2c-bus opened...')

print('====================================')
print('Kalman-Filter mit SRF02 Miniprogramm')
print('====================================')

dt_meas = 0.1 # min time for US measurement (max echo delay + sensor response time)
dt = 1 # Iteration time (must be bigger or equal to dt_meas)
x_start = np.array([[2000],[0]])
P_start = np.diag([1000, 1000])
F = np.array([[1, dt],[0, 1]])
B = 0
sigma_z = 5
R = np.array([[sigma_z**2]])
H = np.array([[1., 0.]])
Q_var = 9

def Q_zero_mean(dim, dt=1., var=1):
    if not (dim == 2):
        raise ValueError("dim must be 2")
    if dim == 2:
        Q = np.array([[.25*dt**4, .5*dt**3],
             [ .5*dt**3,    dt**2]])
    return Q*var

def predict(x, P, F, Q, B, u=0):
    '''Kalman Filter predict step'''
    x_prior = np.dot(F, x)+ np.dot(B, u)
    P_prior = np.dot(F, P).dot(F.T) + Q # A.dot(B) is the same as np.dot(A,B)
    return x_prior,P_prior 

def update(x, P, H, R, z):
    '''Kalman Filter update step'''
    S = np.dot(H, P).dot(H.T) + R
    K = np.dot(P, H.T).dot(inv(S))
    y = z - np.dot(H, x)
    x_post = x + np.dot(K, y)
    P_post = P - np.dot(K, H).dot(P)  
    return x_post, P_post

Q = Q_zero_mean(dim=2, dt=dt, var=Q_var)
print('Variance Process x Q_00 = {:.4f}'.format(Q[0,0]))


try:
    x = x_start
    P = P_start
    print('d vorh +/-Std  d mess +/-Std  d korr +/-Std   v korr +/-Std')
    while True:
        i2c.write_byte_data(0x70, 0, 0x52)
        time.sleep(dt_meas)
        val = i2c.read_word_data(0x70, 0x02)
        val = ((val << 8) & 0xFF00) + (val >> 8)
        x_prior,P_prior = predict(x, P, F, Q, B, 0)
        x_post,P_post = update(x_prior, P_prior, H, R, val)
        print('{:6.0f} +/-{:3.0f}  {:6.0f} +/-{:3.0f}  {:6.0f} +/-{:3.0f}  {:6.0f} +/-{:3.0f}'.format
              (x_prior[0,0],np.sqrt(P_prior[0,0]),val,sigma_z,x_post[0,0],
               np.sqrt(P_post[0,0]),x_post[1,0],np.sqrt(P_post[1,0])))
        x = x_post
        P = P_post
        time.sleep(max(0,dt - dt_meas))
except KeyboardInterrupt:
    print('')
    i2c.close()
    print('...i2c-bus closed.')
    print('Byebye...')

