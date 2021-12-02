#! /usr/bin/env python

     


import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

import threading

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_moveit_eg6', anonymous=True)

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
        self.box_name = 'Box_0'
        
        self._group.set_planning_time(5)
        #self.joint_tolerance=self._group.get_goal_joint_tolerance()
        self._group.set_num_planning_attempts(300)
        self._group.set_goal_joint_tolerance(0.0001)
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

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
        #0.8 9 0 0 0 -1.571
    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()
        print(list_joint_values)
        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')
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
    
    
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')



package=None               
def callback(data):
    for info in data.models:
        if (info.type in ["packagen00","packagen01","packagen02","packagen10","packagen11","packagen12","packagen20","packagen21","packagen22"]):
            global package
            package=info
            break
            
def control_conveyer(ur5):
    #start conveyor belt   
    ur5.conveyor_belt(100)
    while(package==None or package.pose.position.y>0.05):
            pass
    ur5.conveyor_belt(0) 
    while(package!=None):
        pass
        
def main():
    ur5 = Ur5Moveit("ur5_2")
    
    
    
    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage, callback)
    
    thread = threading.Thread(name="conveyer_handler",
                              target=control_conveyer,
                              args=(ur5,))
    thread.start()
    
    colour_bin={"red":0,"yellow":1,"green":2}
  
    joint_angles_red = [math.radians(86),
                          math.radians(-47),
                          math.radians(115),
                          math.radians(-156),
                          math.radians(-90),
                          math.radians(0)]

    # Position to place blue box
    joint_angles_yellow = [math.radians(-0),
                          math.radians(-18),
                          math.radians(54),
                          math.radians(-126),
                          math.radians(-93),
                          math.radians(0)]

    # Position to place green box
    joint_angles_green = [math.radians(-110),
                          math.radians(-37),
                          math.radians(88),
                          math.radians(-140),
                          math.radians(-90),
                          math.radians(-20)]
    joint_angle=[joint_angles_red,joint_angles_yellow,joint_angles_green]
    
    #x -0.762090  y < 0.05  
    pick_position_angle =[math.radians(168),
                          math.radians(-44),
                          math.radians(65),
                          math.radians(-111),
                          math.radians(-90),
                          math.radians(0)]
    
    ur5.hard_set_joint_angles(pick_position_angle,3)
    file_name = "ip.yaml"
    file_path = ur5._file_path + file_name
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
         
    c=["r","y","g"]
    for i in range(3):
        while(package==None):
            pass      
               
        ur5.vacuum_gripper(True)
        
        ur5.box_name="box0"
        ur5.add_box(-0.762,0.98,0.0)    #x,z,y
        ur5.attach_box()
        ur5.hard_set_joint_angles(joint_angle[i],3)
        
        file_name = "p{}.yaml".format(c[i])
        file_path = ur5._file_path + file_name
        with open(file_path, 'w') as file_save:
            yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
            
        ur5.detach_box()
        ur5.vacuum_gripper(False)
        ur5.remove_box("box0")
        
        ur5.hard_set_joint_angles(pick_position_angle,1)
        
        file_name = "{}p.yaml".format(c[i])
        file_path = ur5._file_path + file_name
        with open(file_path, 'w') as file_save:
            yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
        
        global package
        package=None
        
    del ur5
            
                          
    


if __name__ == '__main__':
    main()


