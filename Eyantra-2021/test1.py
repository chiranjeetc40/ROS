#! /usr/bin/env python
##
# This script pick and place package In respective bin based on colour of package using UR5_arm_2.
# first detect all package colour then start conveyer and when any package come at grabbing position under logical camera,
# conveyer stops and colour of box is known by using name of pckage as key and get colour from color_package_dict and place on respective bin.
##

import rospy
#import script written for colour detection and ur5_class 
import ur5_class 
from color_detection import Camera

import threading
from pkg_vb_sim.msg import LogicalCameraImage


#global variable for storing package position and name of package present under logical camera 2 at fix position                
package_position=None  
package_name=None             

""" This is callback function called whenever data is published on /eyrc/vb/logical_camera_2 topic.
    If any package with predefined name is present under logical camera then first package name and Y position is assigned to global variable."""
def callback(data):
    for info in data.models:
        if (info.type in ["packagen00","packagen01","packagen02","packagen10","packagen11","packagen12","packagen20","packagen21","packagen22"]):
            global package_position,package_name
            package_position=info.pose.position.y
            package_name=info.type
            break
            
""" This function control conveyer belt.
    If any package is at grabbing position then it stops and if not then it keep running."""            
def control_conveyer(ur5):
    
    while(1):
        #start conveyor belt at max speed and run till package not at grabbing position
        ur5.conveyor_belt(100)
        while(package_name==None or package_position>0.08):
                pass
        #stop conveyor belt till package is at grabbing position
        ur5.conveyor_belt(0) 
        while(package_name!=None and package_position<0.02):
            pass
        
def main():
    #initilize node 
    rospy.init_node('node_ur5_2_pick_place', anonymous=True)
    ur5 = ur5_class.Ur5Moveit("ur5_2")
    #wait for spwaning of box
#    rospy.sleep(3)
#    # detect colour of all package and store in  color_package_dict which map package name to colour
#    detactColour=Camera()
#    color_package_dict=detactColour.colour_pacakge()
#    
#   
#    
#    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage, callback)
#    
#    thread = threading.Thread(name="conveyer_handler",
#                              target=control_conveyer,
#                              args=(ur5,))
#    thread.start()
#    
#    
#    
#    print(color_package_dict)
#    del detactColour
    package_colour=['r','g','y']
    ur5.moveit_hard_play_planned_path_from_file('ip.yaml' , 2)   
    for i in range(3):
        
        
        trajectory_name="p{}.yaml".format(package_colour[i])
        ur5.moveit_hard_play_planned_path_from_file(trajectory_name , 2)
        ur5.vacuum_gripper(False)
        
        trajectory_name="{}p.yaml".format(package_colour[i])
        ur5.moveit_hard_play_planned_path_from_file(trajectory_name , 2)   
        
    del ur5


if __name__ == '__main__':
    main()



