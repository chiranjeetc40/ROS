#!/usr/bin/env python2


import rospy

from pratice.srv import WordCount,WordCountResponse	#here pratice is package name in which service generated

def count_words(request):
    print("hiii")
	return WordCountResponse(len(request.word.split()))
	# or return len(request.words.split())  or return [len(request.words.split())] or dictionary
	
rospy.init_node('service_server')
service = rospy.Service('word_count', WordCount, count_words)
rospy.spin()
