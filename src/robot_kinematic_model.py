'''
For our model we have a speed and angular velocty



'''
import math
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import time


#init the plotting stuff
plt.axes()
plt.show(block = False)

#Matrix of markers
Markers = np.array([[0,0],[0,1],[.5,-.1],[.1,.5]])
num_of_markers = Markers.shape[0]

#plotting function
def plot_robot(x,y,theta):
	#Plot the camera points
	Bx = x+(max_range*math.cos(theta+(fov/2)))
	By = y+(max_range*math.sin(theta+(fov/2)))
	Cx = x+(max_range*math.cos(theta-(fov/2)))
	Cy = y+(max_range*math.sin(theta-(fov/2)))
	circleB = plt.Circle((Bx, By), radius=0.01, fc='r')
	circleC = plt.Circle((Cx, Cy), radius=0.01, fc='r')
	plt.gca().add_patch(circleB)
	plt.gca().add_patch(circleC)
	
	#Plot the markers
	for i in range(0,num_of_markers):
		circle = plt.Circle((Markers[i][0], Markers[i][1]), radius=0.01, fc='b')
		plt.gca().add_patch(circle)
	
	#Plot the robot
	circle = plt.Circle((x, y), radius=0.04, fc='y')
	plt.gca().add_patch(circle)
	plt.axis([-.5,1,-.5,1])
	plt.draw()
	plt.cla()
	return

	
fov = 1.22 #The cameras fov in rad
max_range = .7 #Max effective range of camera
#This function takes in the robots current position and returns the distance and angle to the nearest marker
def simulate_camera(Ax,Ay,theta):
	#Get the boundary points of the camera so as to define a triangluar field of view for the robot
	Bx = Ax+(max_range*math.cos(theta+(fov/2)))
	By = Ay+(max_range*math.sin(theta+(fov/2)))
	Cx = Ax+(max_range*math.cos(theta-(fov/2)))
	Cy = Ay+(max_range*math.sin(theta-(fov/2)))
	
	#Find all the markers within the field of view
	lowest_rho = 10000
	lowest_alpha = 0
	rho = 1000000
	
	for i in range(0,num_of_markers):
		if in_triangle(Ax,Ay,Bx,By,Cx,Cy,Markers[i][0],Markers[i][1]):
			rho = math.sqrt((Markers[i][0]-Ax)*(Markers[i][0]-Ax)+(Markers[i][1]-Ay)*(Markers[i][1]-Ay)) #sqrt(x^2+y^2)
			alpha = (math.atan((Markers[i][1]-Ax)/(Markers[i][0]-Ay))-theta)%(math.pi/2) #atan(y/x)
			if(rho < lowest_rho and (Markers[i][0]-Ay)!=0):
				lowest_rho = rho
				lowest_alpha = alpha


	print lowest_rho
	print lowest_alpha
	return

#Returns whether or not P is in the triangle defined by A,B,C
#If all the dets are positive or all neg then the point is inside the triangle
def in_triangle(Ax,Ay,Bx,By,Cx,Cy,Px,Py):
	count = 0
	
	if triangle_det(Ax,Ay,Bx,By,Px,Py) >= 0:
		count=count+1
	else:
		count = count-1
		
	if triangle_det(Bx,By,Cx,Cy,Px,Py) >= 0:
		count=count+1
	else:
		count = count-1
		
	if triangle_det(Cx,Cy,Ax,Ay,Px,Py) >= 0:
		count=count+1
	else:
		count = count-1
		
	return (count == 3)or(count == -3)

#Returns the determinant of a triangle
def triangle_det(x0,y0,x1,y1,x2,y2):
	num = (.5)*(x1*y2 - y1*x2 -x0*y2 + y0*x2 + x0*y1 - y0*x1)
	return num


#X is our pose matrix x,y,theta
X =np.array([.0,0,0])

#Init the position and orientation covariance uncertainty
P = np.array([[.01,0,0],[0,.01,0],[0,0,(math.pi)/2]])

#I think these are the error models for the camera and odometry
Q = np.array([[.015,0],[0,(math.pi)/150]])
R = np.array([[.01,0],[0,(math.pi)/180]])

while 1:
	dt =.2
	v = .3
	omega = math.pi/5

	v_corrected = v
	omega_corrected = omega

	theta = X[2]
	A = np.array([[1.,0,-v_corrected*math.sin(theta)*dt],[0,1.,v_corrected*math.cos(theta)*dt],[0.,0,1]])
	W = np.array([[math.cos(theta)*dt,0],[math.sin(theta)*dt,0],[0,dt]])
	
	X =np.add(X, np.array([v_corrected*math.cos(theta)*dt,v_corrected*math.sin(theta)*dt,omega_corrected*dt]))
	
	plot_robot(X[0],X[1],X[2])
	
	#time.sleep(dt)

	#P = A*P*A' + W*Q*W'
	P = (np.dot(np.dot(A,P),A.transpose())) + (np.dot(np.dot(W,Q),W.transpose()))
	
	simulate_camera(X[0],X[1],X[2])
	
	#print(P)
