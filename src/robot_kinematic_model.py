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

def plot_robot(x,y,theta,real_x,real_y,real_theta):
	#Plot the camera points
	Bx = real_x+(max_range*math.cos(real_theta+(fov/2)))
	By = real_y+(max_range*math.sin(real_theta+(fov/2)))
	Cx = real_x+(max_range*math.cos(real_theta-(fov/2)))
	Cy = real_y+(max_range*math.sin(real_theta-(fov/2)))
	circleB = plt.Circle((Bx, By), radius=0.01, fc='r')
	circleC = plt.Circle((Cx, Cy), radius=0.01, fc='r')
	plt.gca().add_patch(circleB)
	plt.gca().add_patch(circleC)
	
	#Plot the markers
	for i in range(0,num_of_markers):
		circle = plt.Circle((Markers[i][0], Markers[i][1]), radius=0.01, fc='b')
		plt.gca().add_patch(circle)
	
	#Plot the actual estimated location
	circle = plt.Circle((x, y), radius=0.04, fc='g')
	plt.gca().add_patch(circle)
	#Plot the actual location
	circle_real = plt.Circle((real_x, real_y), radius=0.04, fc='y')
	plt.gca().add_patch(circle_real)
	
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
	
	#Iterate through all the markers within the field of view and return the distance and angle to the closest one
	for i in range(0,num_of_markers):
		if in_triangle(Ax,Ay,Bx,By,Cx,Cy,Markers[i][0],Markers[i][1]):
			#Find the distance to the marker using the distance formula
			rho = math.sqrt((Markers[i][0]-Ax)*(Markers[i][0]-Ax)+(Markers[i][1]-Ay)*(Markers[i][1]-Ay)) #sqrt(x^2+y^2)
			#Find the angle to the marker
			alpha = math.atan2(Markers[i][1]-Ay,Markers[i][0]-Ax)
			
			#Convert the angle to the marker to an angle between 0 and 2pi rads
			if(alpha < 0):
				alpha = (2*math.pi)+alpha
				
			#Convert the robots current angle to an angle between 0 and 2pi rads
			if (theta >= 2*math.pi):
				theta = theta%(2*math.pi)
			elif (theta < 0):
				while (theta < -2*math.pi):
					theta = theta + 2*math.pi
				theta = 2*math.pi + theta
				
			#We need to increase the angle of the marker if the robot is near 2pi rads(think if the robot is looking at 350degs and the marker is at 0degs, the marker should be interpreted at 360degs)
			if(theta+(fov/2) >= alpha+(2*math.pi)):
				alpha = alpha + 2*math.pi
			#We need to decrease the angle of the marker if the robot has a small degree near 0
			elif(theta-(fov/2) <= alpha-(2*math.pi)):
				alpha = alpha - 2*math.pi
				
			#Finally subtract the angle to the robots angle from the angle to the marker to come up with the relative angle
			alpha = theta - alpha
			
			#If the iterated marker is the closest yet seen, save it	
			if(rho < lowest_rho and (Markers[i][0]-Ay)!=0):
				lowest_rho = rho
				lowest_alpha = alpha
				
	#print 'Distance'
	#print lowest_rho
	#print 'Angle'
	#print lowest_alpha

	#TODO: Courupt the returned values with zero mean gaussain noise based on the R? matrix
	return lowest_rho,lowest_alpha

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
X =np.array([.1,0,0])
X_real = X		#This will track the actual not estimated value of the robot

#Init the position and orientation covariance uncertainty
P = np.array([[.01,0,0],[0,.01,0],[0,0,(math.pi)/2]])

#I think these are the error models for the camera and odometry
Q = np.array([[.015,0],[0,(math.pi)/150]])
R = np.array([[.01,0],[0,(math.pi)/180]])

A = np.identity(3)

while 1:
	dt =.2
	v = .1
	omega = np.pi/10
	v_corrected = v
	omega_corrected = omega
	
	#Prediction Update
	theta = X[2]
	#A = np.array([[1,0,(-v_corrected*np.sin(theta)*dt)],[0,1,(v_corrected*np.cos(theta)*dt)],[0,0,1]])
	W = np.array([[np.cos(theta)*dt,0],[np.sin(theta)*dt,0],[0,dt]])
	
	X =np.add(X, np.array([v_corrected*np.cos(theta)*dt,v_corrected*np.sin(theta)*dt,omega_corrected*dt]))
	X_real=np.add(X_real, np.array([v*np.cos(theta)*dt,v*np.sin(theta)*dt,omega*dt]))
	print X
	
	plot_robot(X[0],X[1],X[2],X_real[0],X_real[1],X_real[2])
	
	time.sleep(dt)

	#P = A*P*A' + W*Q*W'
	P = (np.dot(np.dot(A,P),A.transpose())) + (np.dot(np.dot(W,Q),W.transpose()))
	
	#Get data from camera's - rho is distance and alpha is angle
	(rho, alpha) = simulate_camera(X[0],X[1],X[2])
	#Check to see if the data is valid - if the camera data is bad skip the measurement part of the Kalman filter
	if(rho > 100):	#TODO: Change this in actual implementation to a reasonable number
		continue
	
	#Now we need to guess the location of the maker we are looking at
	#Since being the lefthand part of the camera is a negative angle by convetion subtract theta by that angle
	guess_angle = X[2] - alpha
	#Now use polar to cartesian coordinate converstion to guess the location of the marker we are looking at
	x_guess = rho*math.cos(guess_angle) + X[0]
	y_guess = rho*math.sin(guess_angle) + X[1]
	
	#Now iterate through the list of markers and find the marker closest to the our guess - this should be the exact location of the marker we are looking at
	marker_x = 0
	marker_y = 0
	lowest_dist = 1000000
	for i in range(0,num_of_markers):
		distance_to_marker = math.sqrt(((Markers[i][0]-x_guess)*(Markers[i][0]-x_guess))+((Markers[i][1]-y_guess)*(Markers[i][1]-y_guess)))
		if distance_to_marker < lowest_dist:
			#marker_x and marker_y are coordinates of the marker we think we are looking at
			marker_x = Markers[i][0]
			marker_y = Markers[i][1]
			lowest_dist = distance_to_marker
	
	
	#Measurement Update
	# X[0] = rho*cos(alpha) - marker_x
	# X[1] = rho*sin(alpha) - marker_y
	# X[2] = 0??? I don't think you can infer the angle from the measurement
	#%Calculate H
    #H = [(x(1)-lidar(1))/sqrt((x(1)-lidar(1))^2+(x(2)-lidar(2))^2), (x(2)-lidar(2))/sqrt((lidar(1)-x(1))^2+(lidar(2)-x(2))^2), 0;
    #    			-x(2)/(x(1)^2+x(2)^2), x(1)/(x(1)^2+x(2)^2), 0];
    
    #Calculate H
	H = np.array([[((X[0]-marker_x)/math.sqrt((X[0]-marker_x)*(X[0]-marker_x)+(X[1]-marker_y)*(X[1]-marker_y))),((X[1]-marker_y)/math.sqrt((X[0]-marker_x)*(X[0]-marker_x)+(X[1]-marker_y)*(X[1]-marker_y))),0],[-X[1]/(X[0]*X[0]+X[1]*X[1]),X[0]/(X[0]*X[0]+X[1]*X[1]),0]])
	
	#Calculate V
	V = np.array([[1,0],[0,1]])
    
    #Compute the Kalman Gain
    #K = P*H'*((H*P*H'+V*R*V')^-1);
	inside_par = inv(np.dot(H,np.dot(P,H.transpose())) + np.dot(V,np.dot(R,V.transpose())))
	K = np.dot(P,np.dot(H.transpose(),inside_par))
	
	#Update the state
	Fir = np.array([[rho],[0]])
	Sec = np.array([[math.sqrt((X[0]-marker_x)*(X[0]-marker_x)+(X[1]-marker_y)*(X[1]-marker_y))],[0]])

	X = X + np.dot(K,Fir - Sec).transpose()[0]
	
	#Covariance Update
    #P = (eye(3)-K*H)*P;
	P = np.dot((np.identity(3)-np.dot(K,H)),P)
 
	#print(P)
	
