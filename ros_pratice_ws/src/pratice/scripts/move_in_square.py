#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

def turn90(pub):
	twist.linear.x = 0.0
	twist.angular.z = math.pi/(2*8)
	pub.publish(twist)
	rospy.sleep(8)


def forward(pub):
	twist.linear.x = 0.20
	twist.angular.z = 0.0
	pub.publish(twist)
	rospy.sleep(5)

def stop(pub):
	twist.linear.x = 0.00
	twist.angular.z = 0.0
	pub.publish(twist)
	
if __name__=="__main__":
	rospy.init_node('turtlebot3_square')
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	#turtlebot3_model = rospy.get_param("model", "burger")
	rate = rospy.Rate(10) # 10hz
    
	
	
	twist = Twist()
	twist.linear.x = 0.0
	twist.linear.y = 0.0
	twist.linear.z = 0.0
	twist.angular.x = 0.0
	twist.angular.y = 0.0
	twist.angular.z = 0.0
	i=5
	try:
		while(i>0):
			forward(pub)
			turn90(pub)
			i-=1
		stop(pub)
	except rospy.ROSInterruptException:
		pass
