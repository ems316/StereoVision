'''
For our model we have a speed and angular velocty



'''
import math
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import time

#X is our pose matrix x,y,theta
X =np.array([.0,0,0])

#Init the position and orientation covariance uncertainty
P = np.array([[.01,0,0],[0,.01,0],[0,0,(math.pi)/2]])

#I think these are the error models for the camera and odometry
Q = np.array([[.015,0],[0,(math.pi)/150]])
R = np.array([[.01,0],[0,(math.pi)/180]])

#init the plotting stuff
plt.axes()
plt.show(block = False)

#plotting function
def plot_robot(x,y,theta):
	circle = plt.Circle((x, y), radius=0.04, fc='y')
	plt.gca().add_patch(circle)
	plt.axis([-.5,1,-.5,1])
	plt.draw()
	plt.cla()
	return

while 1:
	dt =.2
	v = .25
	omega = math.pi/5

	v_corrected = v
	omega_corrected = omega

	theta = X[2]
	A = np.array([[1.,0,-v_corrected*math.sin(theta)*dt],[0,1.,v_corrected*math.cos(theta)*dt],[0.,0,1]])
	W = np.array([[math.cos(theta)*dt,0],[math.sin(theta)*dt,0],[0,dt]])
	
	X =np.add(X, np.array([v_corrected*math.cos(theta)*dt,v_corrected*math.sin(theta)*dt,omega_corrected*dt]))
	
	plot_robot(X[0],X[1],X[2])
	
	time.sleep(dt)

	#P = A*P*A' + W*Q*W'
	P = (np.dot(np.dot(A,P),A.transpose())) + (np.dot(np.dot(W,Q),W.transpose()))
	print(P)
