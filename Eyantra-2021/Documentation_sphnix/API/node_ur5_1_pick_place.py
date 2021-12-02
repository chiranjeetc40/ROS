#! /usr/bin/env python
"""
This script pick and place package on conveyer from kiva pod using UR5_arm_1.
"""
import threading
import rospy
from pkg_ros_iot_bridge.msg import msgMqttSub
from node_update_inventory import ManageSheet
from pkg_task5.srv import OrderPlaced, OrderPlacedResponse
from color_detection import Camera
import ur5_class

class DispatchOrder:
    """
    This class implement logic to dispatch the order given.
    Attribute:
    orders_list: list of order placed on mqtt topic.
    orders_completed (dictinary): orders that has been dispatched wih order id as key.
    """
    def __init__(self):
        self.orders_list=[]
        self.orders_completed=[]
        #logic to get file to play is complex make it simple.
                              
        # It store package position with key as colour of that package.
        # When particular package is needed then it's colour can be used to acess file position
        # which can be played to move arm to that position. colour and pose relationship is given by Camera class
        self.colour_pose = {"red": [], "green": [], "yellow": []}
        # Ordered item is maped to colour of package so package colour can be known.
        # This colour name is used to acess position of package.
        self.item_colour_mapping = {"Medicine": "red","Food": "green", "Clothes": "yellow"}
        #position of package is used to acess file name to be played because file name is based on position of package.
        self.pose_file_dict = {"00": ["i0.yaml", "p0.yaml", "0p.yaml"],
                          "01": ["i1.yaml", "p1.yaml", "1p.yaml"],
                          "02": ["i2.yaml", "p2.yaml", "2p.yaml"],
                          "10": ["i3.yaml", "p3.yaml", "3p.yaml"],
                          "11": ["i4.yaml", "p4.yaml", "4p.yaml"],
                          "12": ["i5.yaml", "p5.yaml", "5p.yaml"],
                          "20": ["i6.yaml", "p6.yaml", "6p.yaml"],
                          "21": ["i7.yaml", "p7.yaml", "7p.yaml"],
                          "22": ["i8.yaml", "p8.yaml", "8p.yaml"],
                          "30": ["i9.yaml", "p9.yaml", "9p.yaml"],
                          "31": ["i10.yaml", "p10.yaml", "10p.yaml"],
                          "32": ["i11.yaml", "p11.yaml", "11p.yaml"]}
        
        # It is service handler which accept Empty request and send order information that has been sent.
        # It's logic is also need to improve as order_id is no longer requred in new logic but it's there in message.
        # service handler function run more than one time if task is failed in ur5_2 so fix there.
        serv=rospy.Service('process_placed_order', OrderPlaced, self.handle_placed_order)
        
    def file_name(self,item):
        colour = self.item_colour_mapping[item]
        pose = self.colour_pose[colour][0]
        name = self.pose_file_dict[pose]
        self.colour_pose[colour].pop(0)
        return name
    
    def handle_placed_order(self,request):
        """
        The function  will be called when there is a change of state
        in the Action Client State Machine

        Parameters: goal_handle (dictionary): Goals to be send to action server .
        Returns:
        None.
        """
        # deque from que of placed order
        data = self.orders_completed.pop(0)
        return OrderPlacedResponse(str(data[2]),int(data[1]))
                
    def callback(self,msg,args):
        """
        Callback function is called whenever order is published on subscribed topic.
        Update value in attribute of class.
        Parameters: 
            msg: Order Information.
            args: It's list containing Sheet object for updating sheet.
        Returns: 
            None."""
        
        dictinary = eval(msg.message)
        args[0].update_incoming_order_sheet(dictinary)
        self.orders_list.append([dictinary["item"],dictinary["order_id"],dictinary])
        
       
def main():
    """
    The main function which implement pick place logic."""
    rospy.init_node('node_ur5_1_pick_place', anonymous=True)
    ur5 = ur5_class.Ur5Moveit("ur5_1")
    rospy.sleep(3)
    detact_colour = Camera()
    sheet = ManageSheet()
    orders = DispatchOrder()
        
    # Detect colour of package present in pod with position of package as key to colour value.
    # put all position as value in colour_pose to acess pose based on colour value.
    color_package_dict = detact_colour.colour_pacakge()
    for pose, colour in color_package_dict.items():
            orders.colour_pose[colour].append(pose)
    del detact_colour
    # Update Inventory sheet by sending package info which has been detected by camera.
    thread = threading.Thread(name="inventory sheet", target= sheet.update_inventory_sheet,args=(color_package_dict,))
    thread.start()
    
    # Subscribe to order recieving topic so that order received can be saved and processed.
    rospy.Subscriber(rospy.get_param("config_iot")["mqtt"]["sub_cb_ros_topic"],
                     msgMqttSub, orders.callback, (sheet,))

    #wait for order.
    while not orders.orders_list:
        pass
    # go from initial pose to picking pose.    
    ur5.moveit_hard_play_planned_path_from_file("ipl.yaml", 2)
    order_dispatched = 0        #Count no of order dispatched.    
    while True:
        #if order present then proced to process it.
        if orders.orders_list:
            
            # Sort order so that highest order is processed first.
            # first value of orders_list is item(Medicine,Food,Clothes) which is sorted in priority order 
            # due to alphabet of sorting of them.
            # get high priorit order as it will be 1st after sorting.
            orders.orders_list.sort(reverse=True)
            
            # Remove that order from orders_list
            order_priority = orders.orders_list.pop(0)
            # get file name to play to reach required item. this logic can be chnaged.
            # it return file to go to that position from initial, from placing to package and return to placing position.
            name = orders.file_name(order_priority[0])

            #as arm already is at placing position play place pose to pacakge pose.and pick package.
            ur5.moveit_hard_play_planned_path_from_file(name[1], 4)
            ur5.vacuum_gripper(True)
            #go to placing position and put pacakge.
            ur5.moveit_hard_play_planned_path_from_file(name[2], 4)
            ur5.vacuum_gripper(False)
            
            #update update_orders_dispatched_sheet that order has been dispatched. this can done in threading
            # like just call function and return. if this is implemented in thread then assumption will be that till next function call 
            # data will be updated i.e function had processed previous call.
            sheet.update_orders_dispatched_sheet(order_priority[2])
            
            # Add that order to orders_completed list so that it can be sent to ur5_2 as service.
            orders.orders_completed.append(order_priority)
            order_dispatched += 1       
    del ur5
    
    
if __name__ == '__main__':
    main()
    

