#! /usr/bin/env python
"""
This script pick and place package In respective bin based on colour of package using UR5_arm_2.
first detect all package colour then start conveyer and when any package come at grabbing position
under logical camera,
conveyer stops and colour of box is known by using name of pckage as key and get colour from
color_package_dict and place on respective bin.
8.67 pylint"""

import threading
import rospy
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_task5.srv import OrderPlaced
from node_update_inventory import IotRosBridgeActionClient, ManageSheet
import ur5_class


class ShipOrder:
    """
    This class implement interface to place package from conveyer belt to respective bin.
    
    Attributes: 
    package_position (float):  it store position where package is present in logical camera range.
    package_name (string): It  store name of package detected by logical camera."""
    def __init__(self):
        """
        The constructor for ShipOrder class.
        Initilize all attribute and start conveyer belt in thread.
        """
        self.package_position = None
        self.package_name = None
       
        thread = threading.Thread(name="conveyer_handler", target=self.control_conveyer)
        thread.start()
        
        
    def callback(self,data):
        """
        This is callback function called whenever data is published on /eyrc/vb/logical_camera_2 topic.
        If any package with predefined name is present under logical camera then first package name and
        Y position is assigned to package_name and package_position.
        """
        #implement logic to assign value in such that no need for packages list. 
        # if data.models is not empty get values . 
        packages = ["packagen00", "packagen01", "packagen02", "packagen10",
                    "packagen11", "packagen12", "packagen20", "packagen21", "packagen22"]
        for info in data.models:
            # remove this package name and implement something other like if type is not none
            if info.type in packages:
                self.package_position = info.pose.position.y
                self.package_name = info.type
                break

    
    def control_conveyer(self):
        """
        This function control conveyer belt.
        If any package is at grabbing position then it stops and if not then it keep running.
        """
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        conveyor_belt = rospy.ServiceProxy(
            '/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        while True:
            # start conveyor belt at max speed and run till package not at grabbing position
            conveyor_belt(100)
            while(self.package_name is None or self.package_position > 0.08):
                pass
            # stop conveyor belt till package is at grabbing position
            conveyor_belt(0)
            while(self.package_name != None and self.package_position < 0.1):
                pass


    def get_colour_order_id(self):
        """
        This function return colour of package that is present  in front of logical camera.
        Return colour and order information of that package."""
        
        # Improve below logic.
        rospy.wait_for_service('process_placed_order')
        print("Waiting for server")
        item_colour_mapping = {"Medicine": "red","Food": "green", "Clothes": "yellow"}
        colour_order_id = rospy.ServiceProxy('process_placed_order', OrderPlaced)
        try:
            response = colour_order_id()
            print(response)
            order = eval(response.colour)
            ac_colour = item_colour_mapping[order["item"]]
            return ac_colour,order
            
        except rospy.ServiceException as exc:
            print "Service did not process request: " + str(exc)


def main():
    """
    This main function implement logic to pick and place package in bin. """
    # Initilize node.
    rospy.init_node('node_ur5_2_pick_place', anonymous=True)
    # Initilize ur5 class to send handle ur5_2 arm.
    ur5 = ur5_class.Ur5Moveit("ur5_2")
    # Initilize ManageSheet class to send data to sheet.
    sheet = ManageSheet()
    # Initilize ShipOrder class to ship order.
    orders = ShipOrder()
    
    # Subscribe to logical camera topic so that pacakge received can be processed.
    rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, orders.callback)
    #it is placed as if any order is not present arm has to wait which offset angle due to playing arm fails.
    rospy.sleep(7)

    order_shiped = 0 # Total order shiped.
    #resiting of pose of arm to compenseate for offset due to waiting can be overcome by setting to that pose again 
    # using co-ordinate method to go to that pose. as it will take less time.
    ur5.moveit_hard_play_planned_path_from_file('ip.yaml', 2)
    while True:
        # wait till no package is available in front of arm.and continue if it's there.
        while(orders.package_name is None or orders.package_position > 0.08):
            pass
        # call service to know which package has been recently sent so that colour can be known. 
        # service is called by get_colour_order_id function.
        # colour of package can be known by package name becaus package name and  colour is related but  
        # maybe in some situation service wala is better.
        package_colour, order = orders.get_colour_order_id()
        # As pacakge present in front is processed so set name and position of package 
        # to None to know no package is now present.
        orders.package_position = None
        orders.package_name = None
        # Pick package and place in respective colour bin and palce it.
        #file name to go to pose and come to picking psoe is related to first char of colour name.
        ur5.vacuum_gripper(True)
        trajectory_name = "p{}.yaml".format(package_colour[0])
        ur5.moveit_hard_play_planned_path_from_file(trajectory_name, 5)
        ur5.vacuum_gripper(False)
        
        #update update_orders_shipped_sheet 
        sheet.update_orders_shipped_sheet(order)
        # Go to picking position again 
        trajectory_name = "{}p.yaml".format(package_colour[0])
        ur5.moveit_hard_play_planned_path_from_file(trajectory_name, 5)

    del ur5
    


if __name__ == '__main__':
    main()
