#!/usr/bin/env python
import rospy
from rosbag import Bag
with Bag('catkin_ws/new_vertical1.bag', 'w') as Y:
	for topic, msg, t in Bag('catkin_ws/new_vertical.bag'):
		if topic == '/dreamvu/pal/get/point_cloud':
			Y.write('/points2', msg, t)
		else:
			Y.write(topic, msg, t)
