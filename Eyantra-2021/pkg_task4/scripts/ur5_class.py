import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import rospkg

import yaml
import sys

from std_srvs.srv import Empty

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage
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


        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/{}'.format(arg_robot_name))
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        
        self.vacuum_gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/{}'.format(arg_robot_name), vacuumGripper)
        self.conveyor_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        
        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''
    
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        
        
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')


    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()


    def moveit_play_planned_path_from_file(self, arg_file_name):
        file_path = self._file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
            
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret
        
        
    def moveit_hard_play_planned_path_from_file(self, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_name)
            
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
            
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
   

