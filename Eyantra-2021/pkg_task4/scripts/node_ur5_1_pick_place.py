#! /usr/bin/env python
##
# This script pick and place package on conveyer from kiva pod using UR5_arm_1.
##

import rospy
import ur5_class

''' First play trajectory from initial position to box0 then grab box and place on 
    conveyer by playing trajectory from box to conveyer and keep playing for remain box from conveyer to box
    then box to conveyer'''
    
def main():
    rospy.init_node('node_ur5_1_pick_place', anonymous=True)
    
    ur5 = ur5_class.Ur5Moveit("ur5_1")
    #wait for spwaning of box
    rospy.sleep(4)
    for box_no in range(9):
        name='p{}.yaml'.format(box_no)
        ur5.moveit_hard_play_planned_path_from_file(name , 2)
        ur5.vacuum_gripper(True)
        
        name='{}p.yaml'.format(box_no)
        ur5.moveit_hard_play_planned_path_from_file(name,2)
        ur5.vacuum_gripper(False)
        
    del ur5

if __name__ == '__main__':
    main()

