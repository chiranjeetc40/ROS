#! /usr/bin/env python
##
# This script is to pick and place object and add object in collision planning.
# To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
# This namespace provides us with a `MoveGroupCommander`_ class, a `Planningself._sceneInterface`_ class,
# and a `RobotCommander`_ class.
##


import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospy
import math
from tf.transformations import quaternion_from_euler
from pkg_vb_sim.srv import vacuumGripper


class Ur5Moveit:
    '''Ur5Moveit Class for Pick & Place.'''

    def __init__(self):
        '''Constructor to initialize required attribute.'''

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

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

        self._box_name = "link"

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        # END

    def go_to_pose(self, arg_pose):
        '''This method take pose and set UR5 to that pose if IK exist.'''
        
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
            '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
            '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
        
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

    def add_box(self, timeout=4):
        '''Adding Objects to the Planning self._scene
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        First, we will create a box in the planning scene between box where actual box is placed.
        '''
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.03
        box_pose.pose.position.y = 0.5
        box_pose.pose.position.z = 1.91
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[1]
        box_pose.pose.orientation.z = q[2]
        box_pose.pose.orientation.w = q[3]

        self._scene.add_box(self._box_name, box_pose,size=(0.075, 0.075, 0.075))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=2):
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

    def detach_box(self, timeout=2):
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


def main():

    try:
        ur5 = Ur5Moveit()
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        vacuum_gripper = rospy.ServiceProxy(
            '/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)

        # Position to pick box1
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = 0.03745964732
        ur5_pose_1.position.y = 0.261653548822
        ur5_pose_1.position.z = 1.91774465056
        quaternion = quaternion_from_euler(-1.28748858789,
                                1.32288459639, -1.28740998715)
        ur5_pose_1.orientation.x = quaternion[0]
        ur5_pose_1.orientation.y = quaternion[1]
        ur5_pose_1.orientation.z = quaternion[2]
        ur5_pose_1.orientation.w = quaternion[3]

        # Position to place box1
        ur5_pose_2 = geometry_msgs.msg.Pose()
        ur5_pose_2.position.x = -0.894544499697
        ur5_pose_2.position.y = -0.114014486118
        ur5_pose_2.position.z = 1.16994803193
        quaternion = quaternion_from_euler(-1.1340607464, -
                                0.198661495595, 1.93309398267)
        ur5_pose_2.orientation.x = quaternion[0]
        ur5_pose_2.orientation.y = quaternion[1]
        ur5_pose_2.orientation.z = quaternion[2]
        ur5_pose_2.orientation.w = quaternion[3]

        # Initial Position Angle
        lst_joint_angles_1 = [math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
                          

        #go to position 1 for picking
        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(1)
        #add box to scene
        ur5.add_box()
        
        #pick box by activating gripper
        result = vacuum_gripper(True)
        if result:
            rospy.loginfo("Box is Picked")
        
        #attach box to add in path planning
        result = ur5.attach_box(ur5)
        if result:
            rospy.loginfo("Box is Added to Planning Scene")

        #go to position 2 for placing
        ur5.go_to_pose(ur5_pose_2)
        rospy.sleep(1)

        #place box by deactiving gripper
        result = vacuum_gripper(False)
        if result:
            rospy.loginfo("Box is Placed")
        
        #detach box to remove from path planning
        result = ur5.detach_box(ur5)
        if result:
            rospy.loginfo("Box is Detached from Planning Scene")
        
        #remove box from scene
        ur5.remove_box()
        #go to initial position
        ur5.set_joint_angles(lst_joint_angles_1)
        rospy.sleep(2)
        
        del ur5
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
