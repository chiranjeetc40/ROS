#!/usr/bin/env python


import rospy
import tf

from sensor_msgs.msg import Imu 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('imu_publisher')

imu_pub = rospy.Publisher('imu', Imu, queue_size=10)


current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
   
    i = Imu()
    i.header.stamp = rospy.Time.now()
    i.header.frame_id = 'imu'
    i.orientation = Quaternion(*odom_quat)
    i.angular_velocity = Vector3(0, 0, 0)
    i.linear_acceleration=Vector3(0, 0, -10)
    imu_pub.publish(i)

    last_time = current_time
    r.sleep()
