#!/usr/bin/env python
"""
ROS Node - for updating Inventory sheet.
"""

import time
import datetime
import rospy
import actionlib
from pkg_ros_iot_bridge.msg import msgRosIotAction, msgRosIotGoal, msgRosIotResult


class IotRosBridgeActionClient(object):
    """
    This is a class to send goal to action server which act as bridge between IOT and ROS.

    Attributes:
        _ac: Action Client Initialization .
        goal_handles: Dictionary to Store all the goal handels.
        mqtt_sub_topic: topic to subscribe where Mqtt message is published.
        http_sub_topic: spread_sheet_id of google sheet.
    """

    def __init__(self):
        """
        The constructor for IotRosBridgeActionClient class.

        Initilize all attribute and wait_for_server to start.
        """
        self._ac = actionlib.ActionClient("/action_iot_ros", msgRosIotAction)
        self.goal_handles = {}
        param_config_iot = rospy.get_param("config_iot")
        self.mqtt_sub_topic = param_config_iot["mqtt"]["sub_cb_ros_topic"]
        self.http_sub_topic = param_config_iot["google_apps"]["spread_sheet_id"]
        self.http_sub_topic_eyrc = param_config_iot['google_apps']['submission_spread_sheet_id']
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")


    def on_transition(self, goal_handle):
        """
        The function  will be called when there is a change of state
        in the Action Client State Machine

        Parameters:
            goal_handle (dictionary): Goals to be send to action server .
        Returns:
            None.
        """
        # from on_goal() to on_transition(). goal_handle generated by
        # send_goal() is used here.
        result = msgRosIotResult()

        index = 0
        for i in self.goal_handles:
            if self.goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo(
            "Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())

            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_success:
                rospy.loginfo(
                    "Goal successfully completed. Client Goal Handle #: " +
                    str(index)
                )
            else:
                rospy.loginfo(
                    "Goal failed. Client Goal Handle #: " + str(index))


    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        """
        The function  is used to send Goals to Action Server.

        Parameters:
            arg_protocol (string): protocol used for message(mqtt/http).
            arg_mode (string): publish/subscribe to particular topic.
            arg_topic (string): Topic name for subscribing/publishing message.
            arg_message (string): Message content to be published.
        Returns:
            goal_handle.
        """
        goal = msgRosIotGoal()
        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Sending goal.")

        # self.on_transition - It is a function pointer to a function which will be called when
        # there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal, self.on_transition, None)
        return goal_handle


class ManageSheet(object):
    """
    This is a class to update different google sheet in defined format.

    Attributes:
        iot_obj: IotRosBridgeActionClient object to send goal to Action server.
        inventory_sheet_data: Initial data for inventory sheet.
        incoming_order_sheet_data: Initial data for incoming_order sheet.
        orders_dispatched_sheet_data: Initial data for dispached_order sheet.
        orders_shipped_sheet_data: Initial data for shipped_order sheet.
        colour_item_priority_cost (dictionary): It is dictionary which map colour of package
                                                to it's property like what item it is priority
                                                and price of it
        item_priority_cost (dictionary): It map item to it's priority and price
        order_record: Previous order record. order_id is key.
    """

    def __init__(self):
        """
        The constructor for IotRosBridgeActionClient class.

        Initilize all attribute and wait_for_server to start.
        """
        self.iot_obj = IotRosBridgeActionClient()
        self.inventory_sheet_data = {"id": "Inventory", "Team Id": "VB_1544",
                                     "Unique Id": "AnChRiVi", "Quantity": 1, }
        self.incoming_order_sheet_data = {"id": "IncomingOrders", "Team Id": "VB_1544",
                                          "Unique Id": "AnChRiVi", }
        self.orders_dispatched_sheet_data = {"id": "OrdersDispatched", "Team Id": "VB_1544",
                                             "Unique Id": "AnChRiVi", }
        self.orders_shipped_sheet_data = {"id": "OrdersShipped", "Team Id": "VB_1544",
                                          "Unique Id": "AnChRiVi", }

        self.colour_item_priority_cost = {"red": ["Medicine", "HP", 500],
                                          "green": ["Food", "MP", 300],
                                          "yellow": ["Clothes", "LP", 200], }
        self.item_priority_cost = {"Medicine": ["HP", 500,1], "Food": ["MP", 300,3],
                                   "Clothes": ["LP", 200,5]}
        self.order_record = {}

    def update_inventory_sheet(self, color_package_dict):
        """
        This function update inventory sheet.

        Parameters:
            color_package_dict (dictionary): Colour of all package with position as key.
        Returns:
            None.
        """
        # First capital colour name,position of package,month in no january
        # 01,year last two word
        date_time = time.localtime()
        date_time = str(date_time[1]).zfill(2) + str(date_time[0])[2:]
        goal_no = 0
        goal_no_eyrc=120
        packages = ["00", "01", "02", "10", "11", "12", "20", "21", "22", "30", "31", "32"]
        for position in packages:
            colour=color_package_dict[position]
            #for position, colour in color_package_dict.items():
            sku = colour[0].upper() + position + date_time
            storage_no = "R{}C{}".format(position[0], position[1])

            self.inventory_sheet_data["SKU"] = sku
            self.inventory_sheet_data["Item"] = self.colour_item_priority_cost[colour][0]
            self.inventory_sheet_data["Priority"] = self.colour_item_priority_cost[
                colour
            ][1]
            self.inventory_sheet_data["Storage Number"] = storage_no
            self.inventory_sheet_data["Cost"] = self.colour_item_priority_cost[colour][2]

            self.iot_obj.goal_handles[goal_no] = self.iot_obj.send_goal(
                "http", "NA", self.iot_obj.http_sub_topic, str(self.inventory_sheet_data))
            
#            self.iot_obj.goal_handles[goal_no_eyrc] = self.iot_obj.send_goal(
#                "http", "NA", self.iot_obj.http_sub_topic_eyrc, str(self.inventory_sheet_data))
#            goal_no_eyrc+=1
            goal_no += 1
            rospy.sleep(2)      #2  time for sheet to get updated

    def update_incoming_order_sheet(self, message):
        """
        The function  is used to update Income order sheet.

        Parameters:
            order_dict (dictionary): order given on mqtt topic.
        Returns:
            None.
        """

        self.incoming_order_sheet_data["Priority"] = self.item_priority_cost[
            message["item"]][0]
        self.incoming_order_sheet_data["Cost"] = self.item_priority_cost[message["item"]][1]
        self.incoming_order_sheet_data["Order ID"] = message["order_id"]
        self.incoming_order_sheet_data["Order Date and Time"] =   message["order_time"]
        self.incoming_order_sheet_data["Item"] = message["item"]
        self.incoming_order_sheet_data["Order Quantity"] = message["qty"]
        self.incoming_order_sheet_data["City"] = message["city"]
        self.incoming_order_sheet_data["Longitude"] = message["lon"]
        self.incoming_order_sheet_data["Latitude"] =   message["lat"]                 
        self.iot_obj.goal_handles[message["order_id"]] = self.iot_obj.send_goal(
            "http", "NA", self.iot_obj.http_sub_topic, str(self.incoming_order_sheet_data))
        
#        self.iot_obj.goal_handles[message["order_id"]+"E"] = self.iot_obj.send_goal(
#                "http", "NA", self.iot_obj.http_sub_topic_eyrc, str(self.incoming_order_sheet_data))
        

        self.order_record[message["order_id"]] = self.incoming_order_sheet_data
        rospy.sleep(1)

    def update_orders_shipped_sheet(self, order_record):
        """
        The function  is used to update order shipped sheet.

        Parameters:
            order_id (int): order id of order which is shipped.
        Returns:
            None. 2021-01-31 17:33:09
        """
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        date_time = value.strftime('%Y-%m-%d %H:%M:%S')
        
        self.orders_shipped_sheet_data["Shipped Status"] = "Yes"
        self.orders_shipped_sheet_data["Shipped Date and Time"] = date_time
        date = time.localtime()
        self.orders_shipped_sheet_data["Estimated Time of Delivery"] = "{}/{}/{}".format(
                date[2]+self.item_priority_cost[order_record["item"]][2],date[1],date[0])

        self.orders_shipped_sheet_data["City"] = order_record["city"]
        self.orders_shipped_sheet_data["Order ID"] = order_record["order_id"]
        self.orders_shipped_sheet_data["Item"] = order_record["item"]
        self.orders_shipped_sheet_data["Priority"] = self.item_priority_cost[order_record["item"]][0]
        self.orders_shipped_sheet_data["Cost"] = self.item_priority_cost[order_record["item"]][1]

        self.orders_shipped_sheet_data["Shipped Quantity"] = order_record["qty"]
        self.orders_shipped_sheet_data["Order Date and Time"] = order_record["order_time"]
        
        
        self.iot_obj.goal_handles[order_record["order_id"]] = self.iot_obj.send_goal(
            "http", "NA", self.iot_obj.http_sub_topic, str(self.orders_shipped_sheet_data))
            
#        self.iot_obj.goal_handles[order_record["order_id"]+"E"] = self.iot_obj.send_goal(
#            "http", "NA", self.iot_obj.http_sub_topic_eyrc, str(self.orders_shipped_sheet_data))
        rospy.sleep(1)

    def update_orders_dispatched_sheet(self, order_record):
        """
        The function  is used to update order dispached sheet.

        Parameters:
            order_id (int): order id of order which is dispached.
        Returns:
            None."""
            
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        date_time = value.strftime('%Y-%m-%d %H:%M:%S')
        
        self.orders_dispatched_sheet_data["Dispatch Status"] = "Yes"
        self.orders_dispatched_sheet_data["Dispatch Date and Time"] = date_time
        self.orders_dispatched_sheet_data["City"] = order_record["city"]
        self.orders_dispatched_sheet_data["Order ID"] = order_record["order_id"]
        self.orders_dispatched_sheet_data["Item"] = order_record["item"]
        self.orders_dispatched_sheet_data["Priority"] = self.item_priority_cost[order_record["item"]][0]
        self.orders_dispatched_sheet_data["Cost"] = self.item_priority_cost[order_record["item"]][1]
        self.orders_dispatched_sheet_data["Dispatch Quantity"] = order_record["qty"]
        
        self.iot_obj.goal_handles[order_record["order_id"]] = self.iot_obj.send_goal(
            "http", "NA", self.iot_obj.http_sub_topic, str(self.orders_dispatched_sheet_data))
            
#        self.iot_obj.goal_handles[order_record["order_id"]+"E"] = self.iot_obj.send_goal(
#            "http", "NA", self.iot_obj.http_sub_topic_eyrc, str(self.orders_dispatched_sheet_data))

        rospy.sleep(1)
