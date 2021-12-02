#!/usr/bin/env python
"""
This script is used to detect colour
It detect colour based on assumption that size of image will not change
and box position will also not change.
It assign colour to package name as package_name is fixed for fixed position.
"""
from pyzbar.pyzbar import decode, ZBarSymbol
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2


class Camera:
    """
    This is a class implement interface to detect colour of package placed in kiva pod.

    Attributes:
        bridge: CvBridge object for converting raw image to required format .
        image_sub: subscribe topic object where raw image is published.
        no_box: No of box to be detected from start of kiva pod.
        images_place: Pixel value of [start_row,start_colum,end_row,end_colum] for each 12 boxes.
        color_package_dict: Dictionary holding colour of package as value and position as key.
        packages: List of package position.
    """
    
    def __init__(self):
        """
        The constructor for Camera class.
        
        Initilize opencv object,subscribe to topic on which image is published to and,
        initilize attribute with values.
        """
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self.no_box = 12
        self.all_detected = False
        self.images_place = [[0, 110, 1, 110], [0, 110, 180, 300],
                             [0, 110, 370, 490], [160, 300, 1, 110],
                             [160, 300, 180, 300], [160, 300, 370, 490],
                             [320, 450, 1, 110], [320, 450, 180, 300],
                             [320, 450, 370, 490], [470, 600, 1, 110],
                             [470, 600, 180, 300], [470, 600, 370, 490]]
        self.color_package_dict = {"00": None, "01": None, "02": None, "10": None, "11": None,
                                   "12": None, "20": None, "21": None, "22": None, "30": None,
                                   "31": None, "32": None}
        self.packages = ["00", "01", "02", "10", "11", "12", "20", "21", "22", "30", "31", "32"]


    def get_colour(self, img):
        """
        Get colour of boxes in image.
        The function  will be called when there is new data published to subscribed topic.

        Parameters:
            img: image from which package colour has to be detected.
        Returns:
            None.
        """
        
        # Remove unwanted part of image, if size of image change it can change        
        cropped_image = img[305:900, 120:600]
        pacakge_no = 0
        for box_no in range(self.no_box):
            image = cropped_image[
                self.images_place[box_no][0]: self.images_place[box_no][1],
                self.images_place[box_no][2]: self.images_place[box_no][3],]
                
            resize_img = cv2.resize(image, (0, 0), fx=1.75, fy=1.75)
            detections = decode(resize_img, symbols=[ZBarSymbol.QRCODE])
            
            # if colour is detected then append value in list else start again.
            if len(detections) > 0:
                self.color_package_dict[self.packages[pacakge_no]]=str(detections[0].data)
                pacakge_no = pacakge_no + 1
            else:
                rospy.logerr(
                    "\033[94m" + ">>> Can not detect Colour by QR" + "\033[0m")
                break

        if(pacakge_no>=12):
            self.all_detected = True


    def callback(self, data):
        """
        Callback function when new data is published to topic.

        Parameters:
            data : Raw image published on topic .
        Returns:
            None.
        """
        try:
             # Convert Raw image data to image
            cv_image = self.bridge.imgmsg_to_cv2( data, "bgr8")
        except CvBridgeError as error:
            rospy.logerr(error)
        self.get_colour(cv_image)


    def colour_pacakge(self):
        """
        Return Dictionary with each package corresponding to it's colour.
        If all boxes colour got detected then continue and then Unsubscribe to
        topic to stop running callback function.
        """
        while not self.all_detected:
            print("Waiting for detection of package")
            rospy.sleep(0.5)
        self.image_sub.unregister()
        return self.color_package_dict
        
if __name__ == '__main__':
    pass
