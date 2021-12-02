#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point,Quaternion
import tf
from math import radians,copysign,sqrt,pow,pi
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

x=0
y=0
theta=0

def turn90(pub):
	theta_init=theta
	angle=math.pi/2
	twist=Twist()
	twist.linear.x=0.0
	twist.angular.z=0.2
	while (abs((theta - theta_init))<angle):
		pub.publish(twist)
		rospy.Rate(10).sleep()
	stop(pub)


def forward(pub):
	x_init=x
	y_init=y
	twist=Twist()
	twist.linear.x=0.20
	twist.angular.z=0.0
	while math.sqrt((x - x_init)**2 + (y - y_init)**2) <=1:
		print(math.sqrt((x - x_init)**2 + (y - y_init)**2))
		pub.publish(twist)
		rospy.Rate(10).sleep()
	stop(pub)
	
def stop(pub):
	twist.linear.x=0.00
	twist.angular.z=0.0
	pub.publish(twist)

def callback(data):
	global x,y,theta
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	(roll, pitch, theta) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
	
        
if __name__== "__main__":
	rospy.init_node('turtlebot3_square')
	turtlebot3_model=rospy.get_param("model","burger")
	
	pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
	twist=Twist()
	rospy.Subscriber('/turtle1/pose', Odometry, callback)
	
	i=4
	try:
		while(i>0):
			forward(pub)
			turn90(pub)
			i-=1
	except rospy.ROSInterruptException:
		pass
		
"""
position: 
  x: -5.1906228905e-07
  y: 1.160993185e-05
  z: -0.00100739907941
orientation: 
  x: -1.71858553152e-06
  y: 0.00158965017914
  z: 3.58791627884e-05
  w: 0.99999873586
"""
