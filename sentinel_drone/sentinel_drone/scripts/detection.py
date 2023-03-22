#!/usr/bin/env python3
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64, Float32, Int8
import rospkg
import math
import time
import cv2
import numpy as np
import rospy
from typing_extensions import Self
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray


class image_detection():

    def __init__(self):

        rospy.init_node('yellow_detect')  # Initialise rosnode

        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)
        self.img = np.empty([])
        # This will contain your image frame from camera
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)

    # Callback function of camera topic

    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
            # logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05,minNeighbors=5)

            # for (x, y, w, h) in logo:
            #     cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
            #     #print(x + w/2)
            #     #print(y + h/2)

            #     #Pixel coordinates wrt drone
            #     centre_x = x + w/2 - 200
            #     centre_y = 200 - (y + h/2)
            #     self.pixel_to_m(centre_x, centre_y)
            #     #data_for_publishing = Int32MultiArray(data = [centre_x, centre_y])

            # # plt.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
            # cv2.imshow('image',self.img)
            # cv2.waitKey(1)
            # plt.show()
            # plt.clf()


            self.lower = np.array([15, 150, 20])
            self.upper = np.array([45, 255, 255])
            #img = cv2.imread('yellow_detect.jpeg')
            self.image = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
            self.mask = cv2.inRange(self.image, self.lower, self.upper)
            self.contours, self.hierarchy = cv2.findContours(
                self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(self.contours) != 0:
                for i in self.contours:
                    if cv2.contourArea(i) > 500:
                        self.x, self.y, self.w, self.h = cv2.boundingRect(i)
            self.x_coordinate_yellow = self.x + (self.w/2)
            self.y_coordinate_yellow = self.y + (self.h/2)
            return self.x , self.y
            

        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    try:
        image_dec_obj = image_detection()
    except rospy.ROSInterruptException:
        pass
