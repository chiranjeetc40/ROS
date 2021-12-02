#!/usr/bin/env python
##
# This script is used to detect colour 
# It detect colour based on assumption that size of image will not change and box position will also not change.
# It assign colour to package name as package_name is fixed for fixed position
##

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode,ZBarSymbol


class Camera:
  '''Initilize opencv object,subscribe to topic image is published to,
     colour_list to store colour in order of arrangement of box '''
  def __init__(self,topic="/eyrc/vb/camera_1/image_raw",no_box=12):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic, Image,self.callback)
    self.no_box=no_box
    # Pixel value of [start_row,start_colum,end_row,end_colum] for each 12 boxes
    self.images_place=[[0,110,1,110],[0,110,180,300],[0,110,370,490],
                        [160,300,1,110],[160,300,180,300],[160,300,370,490],
                        [320,450,1,110],[320,450,180,300],[320,450,370,490],
                        [470,600,1,110],[470,600,180,300],[470,600,370,490]]
    self.colour_list=[]
    self.color_package_dict={"packagen00":None,"packagen01":None,"packagen02":None,"packagen10":None,"packagen11":None,
                             "packagen12":None,"packagen20":None,"packagen21":None,"packagen22":None,"packagen30":None,"packagen31":None,"packagen32":None}
    self.packages = ["packagen00","packagen01","packagen02","packagen10","packagen11","packagen12","packagen20","packagen21","packagen22","packagen30","packagen31","packagen32"]

  '''Get colour of boxes in image'''
  def get_colour(self, img):
    # Remove unwanted part of image, if size of image change it can change
    croppedImage = img[305:900,120:600]
    # Empty list so new value can be filled
    self.colour_list=[]
    for box_no in range(self.no_box):
        #Get each box by croping from whole image
        Image = croppedImage[self.images_place[box_no][0]:self.images_place[box_no][1], self.images_place[box_no][2]:self.images_place[box_no][3]]
        # Resize image to make it bigger
        resizeImg = cv2.resize(Image,(0,0) , fx=1.75, fy=1.75)
        # Get colour of image by using QR code decoder
        detections = decode(resizeImg, symbols=[ZBarSymbol.QRCODE])
        #if colour is detected then append value in list else append None
        if ( len( detections ) > 0):
            self.colour_list.append(detections[0].data)
        else :
            #self.colour_list.append(None)
            rospy.logerr(
            '\033[94m' + ">>> Can not detect Colour by QR" + '\033[0m')
        
  '''Callback function when new data is published to topic'''
  def callback(self,data):
    try:
      # Conver Raw image data to image
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    self.get_colour(cv_image)
  
  ''' Return Dictionary with each package corresponding to it's colour'''
  def colour_pacakge(self):
    # If all boxes colour got detected then continue and then Unsubscribe to topic to stop running callback function
    while(len(self.colour_list)<self.no_box):
        pass
    self.image_sub.unregister()
    
    for box_no in range(self.no_box):
        self.color_package_dict[self.packages[box_no]] = str(self.colour_list[box_no])
        
    return self.color_package_dict
        
  


