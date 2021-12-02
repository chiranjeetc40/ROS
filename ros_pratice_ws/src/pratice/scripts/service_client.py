#!/usr/bin/env python2

import rospy
from pratice.srv import WordCount
import sys

rospy.init_node('service_client')
rospy.wait_for_service('word_count')
word_counter = rospy.ServiceProxy('word_count', WordCount)
words = ' '.join(sys.argv[1:])
word_count = word_counter(words)		#more than 1 argument and return value canbe there 
print words, '->', word_count.count
