#! /usr/bin/env python
##
# This script is to pick and place object and add object in collision planning.
# To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
# This namespace provides us with a `MoveGroupCommander`_ class, a `Planningself._sceneInterface`_ class,
# and a `RobotCommander`_ class.
##

import rospy
import math
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from tf.transformations import quaternion_from_euler
from pkg_vb_sim.msg import LogicalCameraImage

transform_packages=None

class Ur5Moveit:
    '''Ur5Moveit Class for Pick & Place.'''

    def __init__(self):
        '''Constructor to initialize required attribute.'''
        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)
        # Buffer to store frame published by tf2,for 10s 
        self._tfBuffer = tf2_ros.Buffer(rospy.Duration(1))
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        self._commander = moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        self._robot = moveit_commander.RobotCommander()

        # Instantiate a `Planningself._sceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        self._scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).  In this tutorial the group is the primary
        # arm joints in the Panda robot, so we set the group's name to "panda_arm".
        # If you are using a different robot, change this value to the name of your robot
        # arm planning group.
        # This interface can be used to plan and execute motions:
        self._planning_group = "ur5_1_planning_group"
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._group.set_max_velocity_scaling_factor(1)
        self._box_name = "link"

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        # END

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)
       
    def set_joint_angles(self, arg_list_joint_angles):
        '''This method take angle and set UR5 arm to that angle.'''
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
    
    #Destructor 
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')  
              
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        '''Wait_for_scene_update
        Ensuring Collision Updates Are Received
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        If the Python node dies before publishing a collision object update message, the message
        could get lost and the box will not appear. To ensure that the updates are
        made, we wait until we see the changes reflected in the
        ``get_attached_objects()`` and ``get_known_object_names()`` lists.
         We wait until the updates have been made or ``timeout`` seconds have passed
        '''
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self._scene.get_attached_objects([self._box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self._box_name in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
            return False

    def add_box(self,trans,timeout=0.1):
        '''Adding Objects to the Planning self._scene
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        First, we will create a box in the planning scene between box where actual box is placed.
        '''
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = trans.translation.x
        box_pose.pose.position.y = trans.translation.y
        box_pose.pose.position.z = trans.translation.z
       
        box_pose.pose.orientation.x = trans.rotation.x
        box_pose.pose.orientation.y = trans.rotation.y
        box_pose.pose.orientation.z = trans.rotation.z
        box_pose.pose.orientation.w = trans.rotation.w

        self._scene.add_box(self._box_name, box_pose,size=(0.075, 0.075, 0.075))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=0.1):
        '''Attaching Objects to the Robot.'''
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # This function add object to scene so that it can be included in planning scene as collision object
        grasping_group = 'ur5_1_planning_group'
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(
            self._eef_link, self._box_name, touch_links=touch_links)
        # END_SUB_TUTORIAL

        # We wait for the planning self._scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=0.1):
        '''Detaching Objects from the Robot.'''
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can also detach and remove the object from the planning self._scene:
        self._scene.remove_attached_object(self._eef_link, name=self._box_name)

        # We wait for the planning self._scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=2):
        '''Removing Objects from the Planning self._scene.'''
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can remove the box from the world.
        self._scene.remove_world_object(self._box_name)

        # **Note:** The object must be detached before we can remove it from the world
        # We wait for the planning self._scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
        
    def func_tf_print(self, arg_frame_1, arg_frame_2):
        trans=None
        while(trans==None):
            try:
                trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time(0))
                return trans.transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")
                continue


def callback(data,args):
    for info in data.models:
        if (info.type in ["packagen1","packagen2","packagen3"]):
            target_frame="logical_camera_2_"+info.type+"_frame"
            global transform_packages
            transform_packages=args[0].func_tf_print("ur5_wrist_3_link",target_frame)
            break

def main():
    try:
        ur5 = Ur5Moveit()
        
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        
        vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
        conveyor_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)

        rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage, callback,(ur5,))
        
        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        delta = vacuum_gripper_width + (box_length/2)  # 0.19 in y 
        
        no_of_package=3
        #add waypoint rather than angle to go to that position
        # Position to place red box
        joint_angles_red = [math.radians(85),
                          math.radians(-53),
                          math.radians(75),
                          math.radians(-122),
                          math.radians(-93),
                          math.radians(0)]

        # Position to place blue box
        joint_angles_blue = [math.radians(-90),
                          math.radians(-53),
                          math.radians(75),
                          math.radians(-122),
                          math.radians(-93),
                          math.radians(0)]

        # Position to place green box
        joint_angles_green = [math.radians(0),
                          math.radians(-53),
                          math.radians(75),
                          math.radians(-122),
                          math.radians(-93),
                          math.radians(0)]
        joint_angle=[joint_angles_red,joint_angles_green,joint_angles_blue]

        # Position for initial position to wait
        initial_position_joint = [2.9214130285450803,
                          -1.330324744772649,
                          2.0610400228065533,
                          -2.3005342867085856,
                          -1.5707302761260902,
                          -0.21920965456757813]

        for i in range(no_of_package):
            #initial position to wait for box
            ur5.set_joint_angles(initial_position_joint)
            
            #start conveyor belt
            conveyor_belt(100)
            while(transform_packages==None or transform_packages.translation.x>0.09):
                pass
            conveyor_belt(0) 
            ur5.ee_cartesian_translation(-(transform_packages.translation.z),(transform_packages.translation.x-0.11),-(transform_packages.translation.y-delta))
                
            vacuum_gripper(True)
            ur5.set_joint_angles(joint_angle[i])
            vacuum_gripper(False)
            global transform_packages
            transform_packages=None     
        
        del ur5
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
