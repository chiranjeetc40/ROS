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
        self.P = np.zeros((5,5))

        # Covariance matrix for process (model) noise
        self.V = np.zeros((5,5))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005
        self.V[3,3] = 0.0
        self.V[4,4] = 0.0
        
        # measurement uncertainty covariance
        self.R = np.matrix([[0.1, 0.0, 0.0,    0.0],
               				[0.0, 0.05,0.0,    0.0],
               				[0.0, 0.0, 0.0025, 0.0],
               				[0.0, 0.0, 0.005,  0.0]])
         
        
        self.delta_t=0.01
        
       

       
        self.I = np.identity(5) # identity matrix


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
    def predict(self,X,P,V):
    	
    	X[0]= X[0]+ self.delta_t * X[3] * np.cos(X[2])
    	X[1]= X[1]+ self.delta_t * X[3] * np.sin(X[2])
    	X[2]= X[2]+ self.delta_t * X[4] 
    	X[3]= X[3]
    	X[4]= X[4]
    	#Jacobian matrix for A
    	a13 = -float(self.delta_t * X[3] * np.sin(X[2]))
    	a14 = float(self.delta_t * np.cos(X[2]))
    	
    	a23 = float(self.delta_t * X[3] * np.cos(X[2]))
    	a24 = float(self.delta_t * np.sin(X[2]))
    	JA = np.matrix([[1.0, 0.0, a13, a14, 0.0 ],
                    	[0.0, 1.0, a23, a24, 0.0 ],
                    	[0.0, 0.0, 1.0, 0.0, self.delta_t  ],
                    	[0.0, 0.0, 0.0, 1.0, 0.0 ],
                    	[0.0, 0.0, 0.0, 0.0, 1.0 ]])
                    	
        #P = JA*P*JA.T + V
        P = (JA.dot(P)).dot(JA.T) + V
        return X,P
        
    def update(self,X,P,Z):
    	xl=Z[4]
    	yl=Z[5]
    	hx = np.matrix([[math.sqrt( (X[0]-xl)*(X[0]-xl) + (X[1]-yl)*(X[1]-yl)) ],
                    	[math.atan2(yl-X[1], xl-X[0]) - X[2]],
                    	[X[3]],
                    	[X[4]]])
                    	
        a11= float((X[0]-xl)/(math.sqrt( (X[0]-xl)*(X[0]-xl) + (X[1]-yl)*(X[1]-yl))))
        a12= float((X[1]-yl)/(math.sqrt( (X[0]-xl)*(X[0]-xl) + (X[1]-yl)*(X[1]-yl))))
        a21= float((yl-X[1])/( (X[0]-xl)*(X[0]-xl) + (X[1]-yl)*(X[1]-yl)))
        a22= float(-(xl-X[0])/( (X[0]-xl)*(X[0]-xl) + (X[1]-yl)*(X[1]-yl)))
        
    	JH = np.matrix([[a11, a12, 0.0, 0.0, 0.0],
                        [a21, a22, -1.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0]])
                        
        #S = JH*P*JH.T + self.R
        S = JH.dot(P).dot(JH.T) + self.R 
        #K = (P*JH.T) * np.linalg.pinv(S)
        K = (P.dot(JH.T)).dot(np.linalg.pinv(S))
        # Update the estimate
        y = Z[0:4] - (hx)  
                       
        #X = X + (K*y)
        X = X + K.dot(y)
        # Update the error covariance
        #P = (self.I - (K*JH))*P
        P = (self.I - K.dot(JH)).dot(P)
        return X,P
        
        
    def estimate(self, sens):
    	# YOUR CODE HEREs
    	Z=np.zeros((6,1))
    	Z[2]=sens.vel_trans
    	Z[3]=sens.vel_ang
    	reading=sens.readings
    	"""if(len(reading)!=0):
    		Z[0]=reading[0].range
    		Z[1]=reading[0].bearing
    		Z[4]=reading[0].landmark.x
    		Z[5]=reading[0].landmark.y"""
    	self.x[3]=Z[2]
    	self.x[4]=Z[3]
    	self.x,self.P=self.predict(self.x,self.P,self.V)	
    	if(len(reading)!=0 ):
    		for data in reading:
    			if(data.range>0.1):
    				Z[0]=data.range
    				Z[1]=data.bearing
    				Z[4]=data.landmark.x
    				Z[5]=data.landmark.y
   
    				self.x,self.P=self.update(self.x,self.P,Z)
        
        
	
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
