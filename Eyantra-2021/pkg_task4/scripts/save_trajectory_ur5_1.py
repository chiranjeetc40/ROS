#! /usr/bin/env python
##
# This script is to pick and place package In respective bin based on colour of package.
# 
##


import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import math
import time
import sys

from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg



class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''


        self._group.set_planning_time(0.5)
        #self.joint_tolerance=self._group.get_goal_joint_tolerance()

        self._group.set_goal_joint_tolerance(0.0001)
        # Attribute to store computed trajectory by the planner	
        
        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        # Buffer to store frame published by tf2,for 10s 
        self._tfBuffer = tf2_ros.Buffer(rospy.Duration(1))
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/{}'.format(arg_robot_name))
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        
        self.vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/{}'.format(arg_robot_name), vacuumGripper)
        self.conveyor_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

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
        
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        number_attempts = 0
        flag_success = False
        #val=0.01
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            if(not flag_success):
                rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
                #self._group.set_goal_joint_tolerance(val)
                #val=val+0.01
            rospy.logwarn("attempts: {}".format(number_attempts) )
        #self._group.set_goal_joint_tolerance(self.joint_tolerance)
        
            # self.clear_octomap()
            
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=2):
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
            attached_objects = self._scene.get_attached_objects([self.box_name])
            
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
    
    def add_box(self,x,z,y=-0.41,timeout=2):
        '''Adding Objects to the Planning self._scene
        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        First, we will create a box in the planning scene between box where actual box is placed.
        '''
       
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[1]
        box_pose.pose.orientation.z = q[2]
        box_pose.pose.orientation.w = q[3]
        #0.075
        self._scene.add_box(self.box_name, box_pose,size=(0.15, 0.15, 0.15))
        rospy.sleep(1)
        #return self.wait_for_state_update(box_is_known=True, timeout=timeout)        
    def attach_box(self, timeout=2):
        '''Attaching Objects to the Robot.'''
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # This function add object to scene so that it can be included in planning scene as collision object
        grasping_group = 'manipulator'
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(
            self._eef_link, self.box_name, touch_links=touch_links)
        # END_SUB_TUTORIAL

        # We wait for the planning self._scene to update.
        #return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=2):
        '''Detaching Objects from the Robot.'''
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can also detach and remove the object from the planning self._scene:
        self._scene.remove_attached_object(self._eef_link, name=self.box_name)

        # We wait for the planning self._scene to update.
        #return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, name,timeout=2):
        '''Removing Objects from the Planning self._scene.'''
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can remove the box from the world.
        self._scene.remove_world_object(name)
        
        touch_links='world'
        self._scene.remove_attached_object(touch_links,self.box_name)

        # **Note:** The object must be detached before we can remove it from the world
        # We wait for the planning self._scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=True, timeout=timeout)
    
    def moveit_play_planned_path_from_file(self, arg_file_name):
        file_path = self._file_path  + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret
    
    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')
                      
    def func_tf_print(self, arg_frame_1, arg_frame_2):
        trans=None
        while(trans==None):
            try:
                trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time(0))
                return trans.transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")
                continue
                

def main():
    rospy.init_node('node_ur5_2_save_trajectory', anonymous=True)
    ur5 = Ur5Moveit("ur5_1")
    
    
    
    joint_angles_0 = [math.radians(-67.7045216299),
                          math.radians(-75.4140976151),
                          math.radians(9.35257961839),
                          math.radians(-110.63907009),
                          math.radians(-101.829379903),
                          math.radians(179.954944132)]
    joint_angles_1 = [math.radians(-126.548981091),
                          math.radians(-90.7573809403),
                          math.radians(27.5960350546),
                          math.radians(-113.202801766),
                          math.radians(-55.4820100738),
                          math.radians(176.530226895)] 
    joint_angles_2 = [math.radians(-160.179904811),
                          math.radians(-74.3140488982),
                          math.radians(18.169790032),
                          math.radians(-140.361359397),
                          math.radians(-26.3846064709),
                          math.radians(172.615948575)]
    joint_angles_3 =  [math.radians(159.389199879),
                          math.radians(-92.4313918232),
                          math.radians(-42.5892732955),
                          math.radians(143.968831973),
                          math.radians(-19.1610550099),
                          math.radians(-30.9534250229)]
    joint_angles_4 =  [math.radians(-136),
                          math.radians(-103),
                          math.radians(59),
                          math.radians(40),
                          math.radians(43),
                          math.radians(-19)]
    joint_angles_5 =  [math.radians(-161),
                          math.radians(-97),
                          math.radians(82),
                          math.radians(-165),
                          math.radians(-17),
                          math.radians(180)]
    joint_angles_6 =  [math.radians(162),
                          math.radians(-84),
                          math.radians(-113),
                          math.radians(17),
                          math.radians(16),
                          math.radians(180)]
                          
    joint_angles_7 =  [math.radians(-120),
                          math.radians(-116),
                          math.radians(132),
                          math.radians(163),
                          math.radians(-60),
                          math.radians(90)]
    joint_angles_8 = [math.radians(53),
                          math.radians(-85),
                          math.radians(-88),
                          math.radians(174),
                          math.radians(-126),
                          math.radians(30)]
    joint_angles_9 =  [math.radians(-53),
                          math.radians(-91),
                          math.radians(118),
                          math.radians(-28),
                          math.radians(126),
                          math.radians(-82)]
    joint_angles_10 =  [math.radians(-125),
                          math.radians(-65),
                          math.radians(-135),
                          math.radians(-162),
                          math.radians(-51),
                          math.radians(-30)]
    joint_angles_11 =  [math.radians(54),
                          math.radians(-94),
                          math.radians(-123),
                          math.radians(-142),
                          math.radians(-128),
                          math.radians(-10)]
                                                
    joint_angles_initial =  [0,0,0,0,0,0]  
                                                                                                                                                                         
    joint_angle_place =[0.08734748009239812, -2.3911351914684706, -0.8726181518472016, -1.4835814326809267, 1.5707953270751522, -9.920693134635172e-05]
    joint_angles=[joint_angles_0,joint_angles_1,joint_angles_2,joint_angles_3,joint_angles_4,joint_angles_5,joint_angles_6,joint_angles_7,joint_angles_8,joint_angles_9,joint_angles_10,joint_angles_11]
    
    ur5.hard_set_joint_angles(joint_angle_place,3)
    file_name = "delete.yaml"
    file_path = ur5._file_path + file_name
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
   
    #Adding Box in planning scene
    """x=[0.28,0.0,-0.28]*4
    z=[1.9]*3+[1.64]*3+[1.42]*3+[1.19]*3
    box_name=['Box_0','Box_1','Box_2','Box_3','Box_4','Box_5','Box_6','Box_7','Box_8','Box_9','Box_10','Box_11']
    for i in range(12):
        ur5.box_name=box_name[i]
        ur5.add_box(x[i],z[i])
    
    
    ur5.conveyor_belt(11)
    #form all box to placing position with box attached
    #ur5.hard_set_joint_angles(joint_angle_place,1)
    #ur5.print_joint_angles()
    #moveit_play_planned_path_from_file(ur5._file_path,"p8.yaml")
    for i in range(10,11):
        ur5.hard_set_joint_angles(joint_angles[i],1)
        #ur5.moveit_play_planned_path_from_file(ur5._file_path,"p10.yaml")
        ur5.box_name=box_name[i]
        ur5.attach_box()
        
        ur5.vacuum_gripper(True)
        
        file_name = "p{}.yaml".format(i)
        file_path = ur5._file_path + file_name
        with open(file_path, 'w') as file_save:
            yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
            
        ur5.hard_set_joint_angles(joint_angle_place,3)
        ur5.print_joint_angles()
        ur5.detach_box()
        ur5.vacuum_gripper(False)
        ur5.remove_box(box_name[i])
        #conveyor_belt(50)
        
        file_name = "{}p.yaml".format(i)
        file_path = ur5._file_path + file_name
        with open(file_path, 'w') as file_save:
            yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
        
        ur5.add_box(x[i],z[i])
        rospy.sleep(0.5)"""
        
    del ur5



if __name__ == '__main__':
    main()


