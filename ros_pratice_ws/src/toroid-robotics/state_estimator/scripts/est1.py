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
        self.t=0.01
        self.u = np.zeros((2,1))
        self.x = np.zeros((3,1))
        self.P = np.zeros((3,3))
        
        self.F = np.matrix([[1.0, 0.0, 0.0],
							[0.0, 1.0, 0.0],
							[0.0, 0.0, 1.0]])
							
		
		
							
        # Covariance matrix for process (model) noise
        self.V = np.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005
        t=0.01
        
        # measurement uncertainty covariance
        self.R = np.matrix([[0.0025, 0.0],
               				[0.0,0.005]])
         
        
        self.Z=np.zeros((4,1))
        
 
        self.I = np.identity(3) # identity matrix


        self.step_size = 0.01

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
    	a11=(math.cos(self.x[2]))*self.t
    	a21=(math.sin(self.x[2]))*self.t
    	self.G = np.matrix([[a11, 0.0],
							[a21, 0.0],
							[0.0,self.t]])
    	self.x = self.F.dot(self.x) + self.G.dot(self.u)
                    	
        self.P = (self.F.dot(self.P)).dot(self.F.T) + self.V
        
        
        
    def update(self):
    	xl=self.Z[2]
    	yl=self.Z[3]
    	
    	hx = np.matrix([[math.sqrt((self.x[0]-xl)**2 + (self.x[1]-yl)**2) ],
                    	[math.atan2(yl-self.x[1], xl-self.x[0]) - self.x[2]]])
                  	
        a11= float((self.x[0]-xl)/(math.sqrt( (self.x[0]-xl)**2 + (self.x[1]-yl)**2)))
        a12= float((self.x[1]-yl)/(math.sqrt( (self.x[0]-xl)**2 + (self.x[1]-yl)**2)))
        a21= float(-(yl-self.x[1])/( (self.x[0]-xl)**2 + (self.x[1]-yl)**2))
        a22= float((xl-self.x[0])/( (self.x[0]-xl)**2 + (self.x[1]-yl)**2))
        
    	JH = np.matrix([[a11, a12, 0.0],
                        [a21, a22, -1.0]])
                        
        S = JH.dot(self.P).dot(JH.T) + self.R
 
        K = (self.P.dot(JH.T)).dot(np.linalg.inv(S))
       	
        # Update the estimate
        y = self.Z[0:2] - (hx)  
                  
        self.x = self.x + K.dot(y)
        
        # Update the error covariance
        self.P = (self.I - K.dot(JH)).dot(self.P)
        
        
        
    def estimate(self, sens):
    	# YOUR CODE HEREs
    	
    	self.predict()
    	self.u[0] = sens.vel_trans
    	self.u[1] = sens.vel_ang
    	reading=sens.readings
    	if(len(reading)!=0):
    		self.Z[0]=reading[0].range
    		self.Z[1]=reading[0].bearing
    		self.Z[2]=reading[0].landmark.x
    		self.Z[3]=reading[0].landmark.y
    		self.update()
    	
    	
    	
        
        
        
	
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
