#**Objective**
The objective of this task is to implement a Warehouse Management System to sort packages based on incoming customer orders from different cities.

To achieve this team needs to do the following things

1. Identify the colour of any 9 packages, three of each colour, on shelf using Camera#1. Either colour detection or QR decoding or combination of both can be used here.
1. As the packages are identified the teams must update the inventory sheet of Inventory Management Spreadsheet of the warehouse which is a Google Spreadsheet using the 
ROS- IoT bridge developed by the Teams in Task 1.
1. After one minute (Sim Time), a total of 9 orders will be published on /eyrc/vb/<unique_id>/orders MQTT Topic at different intervals. Using ROS-IoT Bridge teams need to get the orders from this MQTT Topic.
1. In case the UR5#1 has multiple orders of different priorities to process, teams need to make sure that High Priority orders are processed as quickly as possible, then Medium Priority and then Low Priority orders 
are processed in order to score maximum points (refer grading section to learn more). Packages of red colour will have High Priority symbolizing Medicines, packages of yellow colour
will have a Medium Priority symbolizing Food and packages of green colour have a Low Priority symbolizing Clothes.
1. Once the package is placed on the conveyor belt, teams need to update the Orders Dispatched sheet in the Inventory Management Spreadsheet which should give the status of the packages picked up by the UR5#1 Arm and send an email notification to the user.
1. Once the conveyor belt takes the packages to UR5#2, the UR5#2 Arm then needs to sort the packages based on the colour of the package identified by Camera#1. For eg. Red Package should go in the Red-Bin and so on.
1. As the UR5#2 Arm sorts the individual packages into the bins based on package colour, the teams will need to update the Orders Shipped sheet of the Warehouse Inventory Mastersheet which should give the status of the packages being picked and dropped in the bins by the UR5#2 Arm.
1. As the run is progressing Teams will have to update the Warehouse Inventory Dashboard in real-time. In the Warehouse Inventory Mastersheet teams can create a separate sheet called Dashboard which can show values from other sheet in the Warehouse Inventory Mastersheet spreadsheet. Teams can use this Dashboard sheet as a JSON endpoint to update the Warehouse Inventory Dashboard in real-time.

