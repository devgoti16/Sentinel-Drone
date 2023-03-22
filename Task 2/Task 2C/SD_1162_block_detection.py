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
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float64, Float32, Int8
import rospkg
import math
import time
import cv2
import numpy as np
import rospy
import cv2


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
        self.Ki = [0, 0, 0]
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


# class color_detection():
#     def __init__(self):
#         self.image_sub = rospy.Subscriber(
#             "/edrone/camera/image_raw", Image, self.image_callback)
#         self.bridge = CvBridge()
#         #self.img = np.empty([])

#     def image_callback(self, data):
#         try:
#             # self.cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
#             # self.lower = np.array([15, 150, 20])
#             # self.upper = np.array([45, 255, 255])
#             #img = cv2.imread('yellow_detect.jpeg')
#             self.cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
#             self.lower = np.array([15, 150, 20])
#             self.upper = np.array([45, 255, 255])
#             self.image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
#             self.mask = cv2.inRange(self.image, self.lower, self.upper)
#             self.contours, self.hierarchy = cv2.findContours(
#                 self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             if len(self.contours) != 0:
#                 for i in self.contours:
#                     if cv2.contourArea(i) > 500:
#                         M = cv2.moments(i)
#                         self.cx = -1
#                         self.cy = -1
#                         if (M['m00'] != 0):
#                             self.cx = int(M['m10']/M['m00'])
#                             self.cy = int(M['m01']/M['m00'])
#                             print((int(cx), int(cy)))
#                             rospy.loginfo("box at : " + self.cx +
#                                         " and " + self.cy)
#             return self.cx, self.cy

#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
    


def main():
    e_drone.pid()
    rospy.loginfo("drone started from : " + str(e_drone.drone_position))

    # position_x = [8.0, 8.0, 8.0, 0.0, 0.0, 0.0, -8.0, -8.0, -8.0]
    # position_y = [8.0, 0.0, -8.0, -8.0, 0.0, 8.0, 8.0, 0.0, -8.0]
    # position_z = [19.0, 19.0, 19.0, 19.0, 19.0, 19.0, 19.0, 19.0]
    # for i in range(0, 8):
    #     e_drone.setpoint = [position_x[i], position_y[i], position_z[i]]
    #     while ((e_drone.drone_position[0] > e_drone.setpoint[0] + 0.15 or e_drone.drone_position[0] < e_drone.setpoint[0] - 0.15) or (e_drone.drone_position[1] > e_drone.setpoint[1] + 0.15 or e_drone.drone_position[1] < e_drone.setpoint[1] - 0.15) or (e_drone.drone_position[2] > e_drone.setpoint[2] + 0.15 or e_drone.drone_position[2] < e_drone.setpoint[2] - 0.15)):
    #         e_drone.pid()
    #         r.sleep()
    #     rospy.loginfo("drone reached " + str(e_drone.setpoint))

    e_drone.setpoint = [-9.4, 7.0, 21.0]
    while (1):
        e_drone.pid()
        r.sleep()


if __name__ == '__main__':

    e_drone = Edrone()
    #obj_detection = color_detection()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        main()
        break
