#! /usr/bin/env python2

import rospy
import time
import actionlib #this is required to use action API 

from pratice.msg import TimerAction, TimerGoal, TimerResult #catkin generate this message type from .action file

def do_timer(goal):		#this is callback function when action is called by giving goal as argument
	start_time = time.time()
	time.sleep(goal.time_to_wait.to_sec())   #here goal have time_to_wait type define in .action file which is converted to second using .to_sec()
	result = TimerResult()				#make object of type TimerResult to return result of action
	result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)  #here TimerResult have time_elapsed,updates_sent type define in .action file
	result.updates_sent = 0			
	server.set_succeeded(result)			#using server result of action is returned to caller
	
rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False) #here server is defined with server name,type,callback function,and default running of server=False(must)
server.start() 	#start server
rospy.spin()
