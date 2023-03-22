#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

        PUBLICATIONS			SUBSCRIPTIONS
        /drone_command			/whycon/poses
        /alt_error				/pid_tuning_altitude
        /pitch_error			/pid_tuning_pitch
        /roll_error				/pid_tuning_roll
                    
                                

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from typing_extensions import Self
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from sentinel_drone.msg import Geolocation
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64, Float32, Int8
import rospkg
import math
import time
import cv2
import numpy as np
import subprocess
from osgeo import gdal
import matplotlib.pyplot as plt

class Edrone():
    """docstring for Edrone"""

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        self.setpoint = [0.0, 0.0, 0.0]

        # Declaring a cmd of message type edrone_msgs and initializing values
        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [18, 18, 36]
        self.Ki = [0, 0, 0.000001]
        self.Kd = [400, 400, 449]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.previous_error = [0.0, 0.0, 0.0]
        self.derivative = [0.0, 0.0, 0.0]
        self.Iterm = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        # self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        # You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        # self.sample_time = 0.060 # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher(
            '/drone_command', edrone_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher(
            '/alt_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher(
            '/roll_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            '/pitch_error', Float64, queue_size=1)

        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',
                         PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------

        # ------------------------------------------------------------------------------------------------------------
        self.arm()  # ARMING THE DRONE

    # Disarming condition of the drone

    def disarm(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.

    def arm(self):

        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        # print(self.drone_position[0])
        # print(self.drone_position[1])
        # print(self.drone_position[2])

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        # This is just for an example. You can change the ratio/fraction value accordingly
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.008
        self.Kd[2] = alt.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.01
        self.Ki[1] = pitch.Ki * 0.001
        self.Kd[1] = pitch.Kd * 0.1

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.01
        self.Ki[0] = roll.Ki * 0.001
        self.Kd[0] = roll.Kd * 0.1

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):

        self.roll_error = self.drone_position[0] - self.setpoint[0]
        self.pitch_error = self.drone_position[1] - self.setpoint[1]
        self.alt_error = (self.drone_position[2] - self.setpoint[2])
        self.error = [self.roll_error, self.pitch_error, self.alt_error]

    # -----------------------------Write the PID algorithm here--------------------------------------------------------------

    # Steps:
    # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
    # 2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
    # 3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
    # 4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
    # 5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
    # 6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
    #																														self.cmd.rcPitch = self.max_values[1]
    # 7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
    # 8. Add error_sum

        # Iterm

        self.Iterm[0] = self.error_sum[0] * self.Ki[0]
        self.Iterm[1] = self.error_sum[1] * self.Ki[1]
        self.Iterm[2] = self.error_sum[2] * self.Ki[2]

        # calculate derivative

        self.derivative[0] = (self.error[0] - self.previous_error[0])
        self.derivative[1] = (self.error[1] - self.previous_error[1])
        self.derivative[2] = (self.error[2] - self.previous_error[2])

    # calculate output

        self.out_roll = int(
            1500 - ((self.Kp[0] * self.error[0]) + (self.Iterm[0]) + (self.Kd[0] * self.derivative[0])))
        self.out_pitch = int(
            1500 + ((self.Kp[1] * self.error[1]) + (self.Iterm[1]) + (self.Kd[1] * self.derivative[1])))
        self.out_throttle = int(
            1500 + ((self.Kp[2] * self.error[2]) + (self.Iterm[2]) + (self.Kd[2] * self.derivative[2])))

    # set limit
        if self.out_roll > 2000:
            self.out_roll = 2000

        elif self.out_roll < 1000:
            self.out_roll = 1000

        if self.out_pitch > 2000:
            self.out_pitch = 2000

        elif self.out_pitch < 1000:
            self.out_pitch = 1000

        if self.out_throttle > 2000:
            self.out_throttle = 2000

        elif self.out_throttle < 1000:
            self.out_throttle = 1000

        self.cmd.rcRoll = self.out_roll
        self.cmd.rcPitch = self.out_pitch
        self.cmd.rcThrottle = self.out_throttle

        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_sum[2] = self.error_sum[2] + self.error[2]

        self.previous_error = self.error

        self.alt_error_pub.publish(self.error[2])
        self.roll_error_pub.publish(self.error[0])
        self.pitch_error_pub.publish(self.error[1])

        self.command_pub.publish(self.cmd)

    # ------------------------------------------------------------------------------------------------------------------------


class object_detection():
    def __init__(self):
        self.image_sub = rospy.Subscriber(
            "/edrone/camera_rgb/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.img = np.empty([])
        global image
        #subprocess.run("gdalwarp","-r", "bilinear", "-s_srs", "EPSG:4326", "-t_srs", "EPSG:4326", "-overwrite", 'task2d.tif', "updated.tif")

    def image_callback(self, data):
        try:
            # self.cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            # self.lower = np.array([15, 150, 20])
            # self.upper = np.array([45, 255, 255])
            #img = cv2.imread('yellow_detect.jpeg')
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            #cv2.imwrite('cv_image.jpg',self.cv_image)
            self.lower = np.array([15, 150, 20])
            self.upper = np.array([45, 255, 255])
            self.image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            self.mask = cv2.inRange(self.image, self.lower, self.upper)
            self.ret, self.thresh = cv2.threshold(self.mask, 127, 255, 0)
            self.contours, self.hierarchy = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.imshow("mask",self.mask)
            #self.img2 = cv2.imread('task2d_updated.tif')

            def objdetectedcall(i):
                M = cv2.moments(i)
                self.cx = -1
                self.cy = -1
                if (M['m00'] != 0):
                    self.cx = int(M['m10']/M['m00'])
                    self.cy = int(M['m01']/M['m00'])
                #print((int(self.cx), int(self.cy)))
                    #cv2.waitKey(3)
                    rospy.loginfo("box at : " + str(self.cx) +
                                  " and " + str(self.cy))
                cmd_list3 = ["gdalwarp","-r", "bilinear", "-s_srs","EPSG:4326", "-t_srs", "EPSG:4326", "-overwrite", "//home/dev-ubuntu/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/task2d.tif","task2d_updated.tif"]
                subprocess.run(cmd_list3)

                self.centre_of_box = (self.cx,self.cy)
                cv2.imwrite('cv_image.jpg',self.cv_image)
                img2 = cv2.imread('task2d_updated.tif') 

                self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

                # ORB Detector
                orb = cv2.ORB_create()
                kp1, des1 = orb.detectAndCompute(self.cv_image, None)
                kp2, des2 = orb.detectAndCompute(img2, None)

                    # img1kp=cv2.drawKeypoints(img1,kp1,color=(0,255,0),flags=0)
                    # img2kp=cv2.drawKeypoints(img2,kp2,color=(0,255,0),flags=0)
                    # cv2.imwrite('m_img1.jpg',img1kp)
                    # cv2.imwrite('m_img2.jpg',img2kp)
                    # Brute Force Matching
                bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                matches = bf.match(des1, des2)
                matches = sorted(matches, key = lambda x:x.distance)

                #img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[50:], img2, flags=2)
                    #plt.imshow(img3),plt.show()

                kp1 = list(kp1)
                kp2 = list(kp2)
                # Initialize lists
                 #contains all thepixel corrdinated of matched featured(keypoints)
                list_kp1 = []
                list_kp2 = []

                for mat in matches:

                    # Get the matching keypoints for each of the images
                    #queryIdx and trainIdx tell you which ORB features match between the first and second image
        #queryIdx - The index or row of the kp1 interest point matrix that matches
        #trainIdx - The index or row of the kp2 interest point matrix that matches
                    img1_idx = mat.queryIdx
                    img2_idx = mat.trainIdx

                    # x - columns
                    # y - rows
                    # Get the coordinates
                    (x1, y1) = kp1[img1_idx].pt
                    (x2, y2) = kp2[img2_idx].pt

                    # Append to each list
                    list_kp1.append((x1, y1))
                    list_kp2.append((x2, y2))
                    #list_kp1 will contain the spatial coordinates of a feature point that matched with the corresponding position in list_kp2

                # Open tif file
                ds = gdal.Open('task2d_updated.tif')
                # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
                xoff, a, b, yoff, d, e = ds.GetGeoTransform()
                
                def pixel2coord(x, y):
                    """Returns global coordinates from pixel x, y coords"""
                    xp = a * x + b * y + xoff
                    yp = d * x + e * y + yoff
                    return(xp, yp)
                
                # get columns and rows of your image from gdalinfo
                # rows = 4001
                # colms = 3994
                # long_lat = []
                # for row in  range(0,rows):
                #         for col in  range(0,colms): 
                #             #print (pixel2coord(col,row)) #longitude , #latitude
                #             a,b = pixel2coord(col,row)
                #             long_lat.append((a,b))
                # print(long_lat)
                kp_lat_long = []   #will contain all the longitude adn latitude of matched feature points


                for i in range(0,len(list_kp2)):
                    kplong,kplat = pixel2coord(list_kp2[i][0],list_kp2[i][1])
                    kp_lat_long.append((kplong,kplat))
                #print(kp_lat_long[1][0],kp_lat_long[1][1])

                #print(kp_lat_long)
                cmd_list = ["gdal_translate"]
                for i in range(len(list_kp1)):
                    cmd_list.append("-gcp")
                    cmd_list.append(str(list_kp1[i][0]))
                    cmd_list.append(str(list_kp1[1][1]))
                    cmd_list.append(str(kp_lat_long[i][0]))
                    cmd_list.append(str(kp_lat_long[i][1]))
                cmd_list.append("-of")
                cmd_list.append("GTiff")
                cmd_list.append("cv_image.jpg")
                cmd_list.append("map-with-gcps.tif")

                # cmd_list = ["gdal_translate","-gcp",str(list_kp1[1][0]),str(list_kp1[1][1]),str(kp_lat_long[1][0]),str(kp_lat_long[1][1]),
                #                             "-gcp",str(list_kp1[2][0]),str(list_kp1[2][1]) ,str(kp_lat_long[2][0]) ,str(kp_lat_long[2][1]),
                #                             "-gcp",str(list_kp1[3][0]),str(list_kp1[3][1]) ,str(kp_lat_long[3][0]),str(kp_lat_long[3][1]),
                #                             "-gcp",str(list_kp1[4][0]),str(list_kp1[4][1]),str(kp_lat_long[4][0]),str(kp_lat_long[4][1]),
                #                             "-gcp",str(list_kp1[5][0]),str(list_kp1[5][1]) ,str(kp_lat_long[5][0]),str(kp_lat_long[5][1]),
                #                             "-gcp",str(list_kp1[6][0]),str(list_kp1[6][1]),str(kp_lat_long[6][0]),str(kp_lat_long[6][1]),
                #                             "-gcp",str(list_kp1[7][0]),str(list_kp1[7][1]) ,str(kp_lat_long[7][0]),str(kp_lat_long[7][1]),
                #                             "-gcp",str(list_kp1[8][0]),str(list_kp1[8][1]),str(kp_lat_long[8][0]),str(kp_lat_long[8][1]),
                #                             "-gcp",str(list_kp1[9][0]),str(list_kp1[9][1]),str(kp_lat_long[9][0]),str(kp_lat_long[9][1]),
                #                             "-gcp",str(list_kp1[10][0]),str(list_kp1[10][1]),str(kp_lat_long[10][0]),str(kp_lat_long[10][1]),
                #                             "-gcp",str(list_kp1[11][0]),str(list_kp1[11][1]),str(kp_lat_long[11][0]),str(kp_lat_long[11][1]),
                #                             "-gcp",str(list_kp1[12][0]),str(list_kp1[12][1]),str(kp_lat_long[12][0]),str(kp_lat_long[12][1]),
                #                             "-of","GTiff", "cv_image.jpg", "map-with-gcps.tif"]
                #print(cmd_list)
                subprocess.run(cmd_list)

                cmd_list2 = ["gdalwarp","-r", "bilinear", "-s_srs","EPSG:4326", "-t_srs", "EPSG:4326", "-overwrite", "map-with-gcps.tif","map-with-gcps-change-reference.tif"]
                subprocess.run(cmd_list2)

                # Open tif file
                ds1 = gdal.Open('map-with-gcps-change-reference.tif')
                # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
                xoff1, a1, b1, yoff1, d1, e1 = ds1.GetGeoTransform()

                def pixel2coord1(x1, y1):
                    """Returns global coordinates from pixel x, y coords"""
                    xp1 = a1 * x1 + b1 * y1 + xoff1
                    yp1 = d1 * x1 + e1 * y1 + yoff1
                    return(xp1, yp1)

                centre_long,centre_lat = pixel2coord1(self.cx,self.cy)
                pub = rospy.Publisher('geolocation', Geolocation, queue_size=10)
                geo_location = Geolocation()
                geo_location.objectid = "object"
                geo_location.lat = centre_lat
                geo_location.long = centre_long
                pub.publish(geo_location)
                rospy.loginfo(geo_location)

                    
                

                
            flag=0
            if len(self.contours) != 0:
                #print(2)
                if flag == 0 :
                    for i in self.contours:
                        if cv2.contourArea(i) > 100 :
                            objdetectedcall(i)
                            flag = 1
                            print(flag)
                        # M = cv2.moments(i)
                        # self.cx = -1
                        # self.cy = -1
                        # if (M['m00'] != 0):
                        #     self.cx = int(M['m10']/M['m00'])
                        #     self.cy = int(M['m01']/M['m00'])
                        #     #print((int(self.cx), int(self.cy)))
                        #     cv2.imshow("frame",self.image)
                        #     rospy.loginfo("box at : " + str(self.cx) + " and " + str(self.cy))
                        # geoprocessing(self.image)
                        # pass
                        else : 
                            flag = 0 

            # return self.cx, self.cy

            # def geoprocessing(self):
            #     from_SRS = "EPSG:4326"
            #     to_SRS = "EPSG:4326"
            #     src = 'crs_updated.tif'
            #     dest = 'updated.tif'
            #     cmd_list = ["gdalwarp", "-r", "bilinear", "-s_srs",from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]
            #     subprocess.run(cmd_list)
            #     from osgeo import gdal

            #     # Open tif file
            #     ds = gdal.Open('aerial.tif')
            #     # GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up.
            #     xoff, a, b, yoff, d, e = ds.GetGeoTransform()

            #     def pixel2coord(x, y):
            #         """Returns global coordinates from pixel x, y coords"""
            #         xp = a * x + b * y + xoff
            #         yp = d * x + e * y + yoff
            #         return(xp, yp)

            #     # get columns and rows of your image from gdalinfo
            #     rows = 36+1
            #     colms = 34+1

            #     for row in  range(0,rows):
            #         for col in  range(0,colms):
            #             print (pixel2coord(col,row)) #longitude , #latitude
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


def main():
    e_drone.pid()
    rospy.loginfo("drone started from : " + str(e_drone.drone_position))

    # position_x = [-5.0,6.5,4.5]
    # position_y = [-6.5,-5.0,-5.0]
    # position_z = [20.0,20.0,20.0]
    # for i in range(0, 3):
    #     e_drone.setpoint = [position_x[i], position_y[i], position_z[i]]
    #     while ((e_drone.drone_position[0] > e_drone.setpoint[0] + 0.15 or e_drone.drone_position[0] < e_drone.setpoint[0] - 0.15) or (e_drone.drone_position[1] > e_drone.setpoint[1] + 0.15 or e_drone.drone_position[1] < e_drone.setpoint[1] - 0.15) or (e_drone.drone_position[2] > e_drone.setpoint[2] + 0.15 or e_drone.drone_position[2] < e_drone.setpoint[2] - 0.15)):
    #         e_drone.pid()
    #         r.sleep()
    #     rospy.loginfo("drone reached " + str(e_drone.setpoint))

    e_drone.setpoint = [-5.0, 6.50, 20.0]
    while (1):
        e_drone.pid()
        r.sleep()


if __name__ == '__main__':

    e_drone = Edrone()
    obj_detection = object_detection()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        main()
        break
