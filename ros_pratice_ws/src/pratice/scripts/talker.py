#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy

from std_msgs.msg import String 
from pratice.msg import Complex



def talker():
    pub = rospy.Publisher('chatter', Complex, queue_size=10)	#In case the node sending the messages is transmitting at a higher rate than
																#the receiving node(s) can receive them, rospy will simply drop any messages
																#beyond the queue_size .
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello Chiranjeet %s" % rospy.get_time()
        
        rospy.loginfo(hello_str)
        msg=Complex()
        msg.real=2.2
        msg.imaginary=3
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
