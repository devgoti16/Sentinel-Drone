#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import numpy as np

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Print "Hello!" to terminal
#print("Hello!")

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('drone_img', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window


def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(0)

  # Define a callback for the Image message


def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        lower = np.array([15,150,20])
        upper = np.array([45,255,255])
        #img = cv2.imread('yellow_detect.jpeg')
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image , lower , upper)
        contours,hierarchy = cv2.findContours(mask ,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            for i in contours:
                if cv2.contourArea(i) > 500:
                    x,y,w,h = cv2.boundingRect(i)
        print(int(x+(w/2)),int(y+(h/2)))  
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    show_image(cv_image)

  # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber(
    "/edrone/camera_rgb/image_raw", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
