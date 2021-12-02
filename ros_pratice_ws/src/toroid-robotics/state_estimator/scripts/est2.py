#!/usr/bin/env python
import math
import numpy as np
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = np.zeros((5,1))
        self.P = np.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = np.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005
        
        
        # measurement uncertainty covariance
        self.R = np.matrix([[0.1, 0.0,],
               				[0.0, 0.05]])

        self.I = np.identity(3) # identity matrix
        self.step_size = 0.01
        self.delta_t=0.01
        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks thein
    # robot can currently observe
    def predict(self):
    	
    	self.x[0]= self.x[0]+ self.delta_t * self.x[3] * np.cos(self.x[2])
    	self.x[1]= self.x[1]+ self.delta_t * self.x[3] * np.sin(self.x[2])
    	self.x[2]= self.x[2]+ self.delta_t * self.x[4] 
    	
    	#Jacobian matrix for A
    	a13 = -float(self.delta_t * self.x[3] * np.sin(self.x[2]))
    	a23 = float(self.delta_t * self.x[3] * np.cos(self.x[2]))
    	b11= float(self.delta_t * np.cos(self.x[2]))
    	b21=float(self.delta_t  * np.sin(self.x[2]))
    	
    	JA = np.matrix([[1.0, 0.0, 0.0],
                    	[0.0, 1.0, 0.0],
                    	[a13, a23, 1.0]])
                    	
       
        self.P = (JA.dot(self.P)).dot(JA.T) + self.V
        
        
    def update(self,Z):
    	xl=Z[2]
    	yl=Z[3]
    	hx = np.matrix([[math.sqrt( (self.x[0]-xl)**2 + (self.x[1]-yl)**2) ],
                    	[math.atan2(yl-self.x[1], xl-self.x[0]) - self.x[2]]])
                    	
        d=(math.sqrt( (self.x[0]-xl)**2 + (self.x[1]-yl)**2))  
        m=(self.x[0]-xl)**2 + (self.x[1]-yl)**2 
        	
        a11= float((self.x[0]-xl)/d)
        a12= float((self.x[1]-yl)/d)
   
        a21= float((yl-self.x[1])/m )
        a22= float(-(xl-self.x[0])/m)
        
    	JH = np.matrix([[a11, a12, 0.0],
                        [a21, a22, -1.0]])
                        
        
        S = (JH.dot(self.P)).dot(JH.T) + self.R 
        
        K = (self.P.dot(JH.T)).dot(np.linalg.inv(S))
        # Update the estimate
        y = Z[0:2] - (hx)  
          
        self.x[0:3] = self.x[0:3] + K.dot(y)
        # Update the error covariance
        self.P = (self.I - K.dot(JH)).dot(self.P)
        
        
        
        
        
    def estimate(self, sens):
    	# YOUR CODE HEREs
    	Z=np.zeros((4,1))
    	reading=sens.readings
    	
    	self.x[3]=sens.vel_trans
    	self.x[4]=sens.vel_ang
    	self.predict()	
 
    	if(len(reading)!=0 ):
    		for data in reading:
    			if(data.range>0.3):
    				Z[0]=data.range
    				Z[1]=data.bearing
    				Z[2]=data.landmark.x
    				Z[3]=data.landmark.y
    				self.update(Z)
    				
       
        
	
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
