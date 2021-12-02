#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point,Quaternion
import tf
#from transform_utils import quat_to_angle,normalize_angle
#from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians,copysign,sqrt,pow,pi

WAFFLE_MAX_LIN_VEL=0.26
WAFFLE_MAX_ANG_VEL=1.82

LIN_VEL_STEP_SIZE=0.01
ANG_VEL_STEP_SIZE=0.1


class Move:
	def __init__ (self):
		rospy.init_node('turtlebot3_square',anonymous=True)
		self.cmd_vel=rospy.Publisher('cmd_vel',Twist,queue_size=10)
		rate=10
		r=rospy.Rate(rate)
		turtlebot3_model=rospy.get_param("model","burger")#X
		rospy.on_shutdown(self.shutdown)
		self.tf_listener=tf.TransformListener()
		#Givetfsometimetofillitsbuffer
		rospy.sleep(2)
		#settheodomframe
		self.odom_frame='/odom'
		try:
			self.tf_listener.waitForTransform(self.odom_frame,'base_footprint',rospy.Time(),rospy.Duration(1.0))
			self.base_frame='/base_footprint'
		except(tf.Exception,tf.ConnectivityException,tf.LookupException):
			try:
				self.tf_listener.waitForTransform(self.odom_frame,'base_link',rospy.Time(),rospy.Duration(1.0))
				self.base_frame='/base_link'
			except(tf.Exception,tf.ConnectivityException,tf.LookupException):
				rospy.loginfo("cannotfindtransformbetween/odomand/base_linkor/base_footprint")
				rospy.signal_shutdown("tfException")

		linear_speed=0.15
		goal_distance=1.0
		angular_speed=0.5
		angular_tolarance=radians(1.0)
		goal_angle=pi/2

		position=Point()
		for i in range(4):
			move_cmd=Twist()
			move_cmd.linear.x=linear_speed
			(position,rotation)=self.get_odom()
			x_start=position.x
			y_start=position.y
			distance=0
			while distance<goal_distance and not rospy.is_shutdown():
				self.cmd_vel.publish(move_cmd)
				r.sleep()
				(position,rotation)=self.get_odom()
				distance=sqrt(pow((position.x-x_start),2)+
				pow((position.y-y_start),2))

		move_cmd=Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)

		move_cmd.angular.z=angular_speed
		last_angle=rotation
		turn_angle=0
		while abs(turn_angle+angular_tolarance)<abs(goal_angle) and not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			r.sleep()
			(position,rotation)=self.get_odom()
			delta_angle=normalize_angle(rotation-last_angle)
			

			turn_angle+=delta_angle
			last_angle=rotation

		move_cmd=Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)

		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

	def get_odom(self):
		try:
			(trans,rot)=self.tf_listener.lookupTransform(
			self.odom_frame,self.base_frame,rospy.Time(0))
		except(tf.Exception,tf.ConnectivityException,tf.LookupException):
			rospy.loginfo("TFException")
			return
		return(Point(*trans),quat_to_angle(Quaternion(*rot)))

	def shutdown(self):
		rospy.loginfo("Stoppingtherobot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

def turn90(pub):
	twist.linear.x=0.0
	twist.angular.z=math.pi/(2*8)
	pub.publish(twist)
	rospy.sleep(8)


def forward(pub):
	twist.linear.x=0.20
	twist.angular.z=0.0
	pub.publish(twist)
	rospy.sleep(5)

def stop(pub):
	twist.linear.x=0.00
	twist.angular.z=0.0
	pub.publish(twist)

"""if__name__=="__main__":
	rospy.init_node('turtlebot3_square')
	pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
	turtlebot3_model=rospy.get_param("model","burger")

	rate=rospy.Rate(10)#10hz
	twist=Twist()
	twist.linear.x=0.0
	twist.linear.y=0.0
	twist.linear.z=0.0
	twist.angular.x=0.0
	twist.angular.y=0.0
	twist.angular.z=0.0
	i=5
	try:
		while(i>0):
			forward(pub)
			turn90(pub)
			i-=1
		stop(pub)
	except rospy.ROSInterruptException:
		pass"""





if __name__== '__main__':
	try:
		OutAndBack()
	except rospy.ROSInterruptException:
		rospy.loginfo("out_and_backnodeterminated.")
			
		
"""def __init__(self):


        super(SquareMoveOdom, self).__init__()

        self.pub_rate = 0.1

    def get_z_rotation(self, orientation):

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        print roll, pitch, yaw
        return yaw
        
    def move_of(self, d, speed=0.5):

        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y

        # Set the velocity forward until distance is reached
        while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
             (self.odom_pose.position.y - y_init)**2) < d and not ros.is_shutdown():

            sys.stdout.write("\r [MOVE] The robot has moved of {:.2f}".format(math.sqrt((self.odom_pose.position.x - x_init)**2 + \
            (self.odom_pose.position.y - y_init)**2)) +  "m over " + str(d) + "m")
            sys.stdout.flush()

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def turn_of(self, a, ang_speed=0.4):

        # Convert the orientation quaternion message to Euler angles
        a_init = self.get_z_rotation(self.odom_pose.orientation)
        print a_init

        # Set the angular velocity forward until angle is reached
        while (self.get_z_rotation(self.odom_pose.orientation) - a_init) < a and not ros.is_shutdown():

            # sys.stdout.write("\r [TURN] The robot has turned of {:.2f}".format(self.get_z_rotation(self.odom_pose.orientation) - \
            #     a_init) + "rad over {:.2f}".format(a) + "rad")
            # sys.stdout.flush()
            # print (self.get_z_rotation(self.odom_pose.orientation) - a_init)

            msg = Twist()
            msg.angular.z = ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not ros.is_shutdown():
            time.sleep(0.1)

        # Implement main instructions
        # self.move_of(0.5)
        self.turn_of(math.pi/4)
        self.move_of(0.5)
        self.turn_of(math.pi/4)
        self.move_of(0.5)
        self.turn_of(math.pi/4)
        self.move_of(0.5)
        self.stop_robot()


"""

