

# source /opt/ros/melodic/setup.bash && export ROS_MASTER_URI=http://master.local:11311 && cd /home/pi/ros_catkin_ws/src/joystick2 && python joystick_not_autonomous.py

# source /opt/ros/melodic/setup.bash
# source /home/pi/ros_catkin_ws/devel_isolated/setup.bash
# cd /home/pi/ros_catkin_ws/src/joystick2 && python joystick_plus_autonomous.py

# https://stackoverflow.com/questions/37373211/update-the-global-variable-in-rospy
# export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2
# export ROS_MASTER_URI=http://master.local:11311

import rospy
import std_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

from pandas import read_csv
from pandas import *
from array import *
import numpy as mp
import math

import os
import time
from datetime import datetime

from gpiozero import MCP3008

class Nodo(object):
    def __init__(self):

        # Params:
        self.x1 = 0.0
        self.y1 = 0.0
        self.throttControl = 67.0
        self.steerControl = 49.0
        self.actual_steer_angle = 0.0
        self.camera_pan = 1000
        self.camera_tilt = 1000

        self.loop_rate = rospy.Rate(5)    # Hertz
        
        # Publishers:
        self.pubr = rospy.Publisher("camera_gimbal_pan_msg", Float32, queue_size=10)
        self.pubs = rospy.Publisher("camera_gimbal_tilt_msg", Float32, queue_size=10)
        self.pubt = rospy.Publisher("throttControl_msg", Float32, queue_size=10)
        self.pubu = rospy.Publisher("steerControl_msg", Float32, queue_size=10)

        
        # Subscribers:
        rospy.Subscriber("dogx1_msg", Float32, self.callback_dogX_GPS)
        rospy.Subscriber("dogy1_msg", Float32, self.callback_dogY_GPS)
        rospy.Subscriber("act_steer_ang", Float32, self.callback_act_steer_ang)


    def callback_dogX_GPS(self, msg):
        self.x1 = msg.data

    def callback_dogY_GPS(self, msg):
        self.y1 = msg.data

    def callback_act_steer_ang(self, msg):
        self.actual_steer_angle = msg.data

    def start(self):
        print("Hello !!")
        n = 0
        j = 0
        df = read_csv('/home/pi/GPS_Way_Points/GPS_waypoints_02.csv')
        waypoints = df.values
        prev_GPS_X_values = [0,0,0,0,0,0]
        prev_GPS_Y_values = [0,0,0,0,0,0]
        
        actual_heading = [0,0]
        norm_actual_heading = [0,0]
        desired_heading = [0,0]
        norm_desired_heading =[0,0]
        current_waypoint = [0,0]
        prev_waypoints = []  # If past (1) or not (0) ... (true or false).
        manual_overide = False
        
        for i in range(20):
            prev_waypoints.append(0)
                
        print("")
        for i in range (18):
            waypointX = waypoints[i,1]
            waypointY = waypoints[i,2]
            print "Waypoint " + str(i) + ": " + str(waypointX) + "," + str(waypointY) + ""
        print("")

        while not rospy.is_shutdown():

            while(True):
                print ("\033[1;33;40m" + "-----------------------------------------------------------------------------------------------------------------------------" + "\033[0;0m")
                print("\033[1;36;40mDOG X1: " + (str)(self.x1) + "\033[0;0m")
                print("\033[1;36;40mDOG Y1: " + (str)(self.y1) + "\033[0;0m")
                print ("------------------------")
                
                prev_GPS_X_values[0] = self.x1
                prev_GPS_Y_values[0] = self.y1
                
                # We can save heading data in an array, which updates itself by shifting values from left to right as machine moves forwards.
                # However, we dont want the heading data to be lost if the machine comes to a standstill.
                # TODO need to add a tolerance of eg +-5 to line 99.
                
                if (prev_GPS_X_values[0] != prev_GPS_X_values[1]) and (prev_GPS_X_values[0] != prev_GPS_X_values[1]):
                    # if (prev_GPS_X_values[0]-1 != prev_GPS_X_values[1]) and (prev_GPS_X_values[0]-1 != prev_GPS_X_values[1]):
                        # if (prev_GPS_X_values[0]+1 != prev_GPS_X_values[1]) and (prev_GPS_X_values[0]+1 != prev_GPS_X_values[1]):
                    for i in range (5):
                        prev_GPS_X_values[5 - i] = prev_GPS_X_values[4 - i]    # Shift the values of the array one slot to the right.
                        prev_GPS_Y_values[5 - i] = prev_GPS_Y_values[4 - i]    # Shift the values of the array one slot to the right.
                print("Array prev_GPS_X_values: " + str(prev_GPS_X_values))
                print("Array prev_GPS_Y_values: " + str(prev_GPS_Y_values))
                print("last prev_GPS_X_value in array: " + str(prev_GPS_X_values[5]))
                
                # The prev_GPS_values arrays can be used to create a heading array over (10 / rospy.Rate) seconds eg 2:
                
                actual_heading[0] = prev_GPS_X_values[0] - prev_GPS_X_values[5]
                actual_heading[1] = prev_GPS_Y_values[0] - prev_GPS_Y_values[5]

                
                
                # The desired heading vector is the next waypoint - the current position.
                # Vectors are being used to preserve negative heading values which would get over-looked if using polar coordinates.
                # To get a meaningful comparison between desired and actual headings, both vectors need to be normalised to set the distance element to '1'.
                # For the time being, set the current waypoint to number 1:
                
                current_waypoint[0] = waypoints[1,1]
                current_waypoint[1] = waypoints[1,2]
                desired_heading[0] = current_waypoint[0] - self.x1
                desired_heading[1] = current_waypoint[1] - self.y1
                print ("------------------------")
                print("Actaul heading vector array: " + str(actual_heading))
                print("Desired heading vector array: " + str(desired_heading))
                print ("------------------------")
                
                # Now normalise the desired heading vector:
                
                # desired_heading_hypotenuse = (((desired_heading[0] +0j)**2 + (desired_heading[1] +0j)**2)**0.5).real
                desired_heading_hypotenuse = math.hypot(desired_heading[0],desired_heading[1])
                
                norm_desired_heading[0] = desired_heading[0]/(desired_heading_hypotenuse+0.1)
                norm_desired_heading[1] = desired_heading[1]/(desired_heading_hypotenuse+0.1)
                
                print("Normalised desired heading vector array: " + str(norm_desired_heading))
                
                # Now normalise the actual heading vector:
                
                # actual_heading_hypotenuse = (((actual_heading[0] +0j)**2 + (actual_heading[1] +0j)**2)**0.5).real +0.00000001
                actual_heading_hypotenuse = math.hypot(actual_heading[0],actual_heading[1])
                
                norm_actual_heading[0] = actual_heading[0]/(actual_heading_hypotenuse +0.00000001)
                norm_actual_heading[1] = actual_heading[1]/(actual_heading_hypotenuse +0.00000001)
                
                print("Normalised actual heading vector array: " + str(norm_actual_heading))
                print ("------------------------")

                angle_rads = np.math.atan2(np.linalg.det([norm_desired_heading,norm_actual_heading]),np.dot(norm_desired_heading,norm_actual_heading))
                nav_angle_degrees = math.degrees(angle_rads)
                
                # Determine which is the current waypoint by calculating which waypoints are behind / in front of machine,
                # or by using the distance left to the waypoint.
                
                
                for i in range(20):
                    current_waypoint[0] = waypoints[i,1]
                    current_waypoint[1] = waypoints[i,2]
                    desired_heading[0] = current_waypoint[0] - self.x1
                    desired_heading[1] = current_waypoint[1] - self.y1
                
                    desired_heading_hypotenuse = math.hypot(desired_heading[0],(desired_heading[1]+0.1))
                
                    norm_desired_heading[0] = desired_heading[0]/desired_heading_hypotenuse
                    norm_desired_heading[1] = desired_heading[1]/desired_heading_hypotenuse
                    
                    # The vector norm_desired_heading is now composed of norm_desired_heading[0] and norm_desired_heading[1]
                
                    angle_rads = np.math.atan2(np.linalg.det([norm_desired_heading,norm_actual_heading]),np.dot(norm_desired_heading,norm_actual_heading))
                    nav_angle_degrees = math.degrees(angle_rads)
                    
                    totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                    
                    if (totalDistance > 150 and (prev_waypoints[i] == 0)):           # Tune the totalDistance greater than value.
                        current_waypoint_number = i
                        prev_waypoints[i-1] = 1
                        break
                        
                print("\033[1;31;40mPrevious waypoint number: " + str(i-1) + " is now set to: " + str(prev_waypoints[i-1]) + "\033[0;0m")
                print("\033[1;31;40mCurrent waypoint number: " + str(i) + "\033[0;0m")
                print("\033[1;31;40mDistance to current waypoint: " + str(round(totalDistance)) + "\033[0;0m")


                # print("i: " + str(i))
                if (i == 19): print("\033[1;31;40mNo valiad waypoints were determined at this time. Try moving machine forwards\033[0;0m")
                        
                # print("Dot product angle in radians: " + str(angle_rads))
                print("\033[1;34;40mDot product angle in degrees: " + str(nav_angle_degrees)  + "\033[0;0m")
                if(nav_angle_degrees > 0): print("\033[1;35;40mMachine needs to turn right \033[0;0m")
                if(nav_angle_degrees < 0): print("\033[1;35;40mMachine needs to turn left \033[0;0m")
                print ("------------------------")
                
                print("Straight ahaed angle = 450")
                print("\033[1;32;40mActual steering angle: " + (str)(self.actual_steer_angle) + "\033[0;0m")
                print ("------------------------")
                
                ##############################################################################################
                #'''
                ## This is where autonomous stuff starts:
                # Check if we have a full buffer for heading:
                # turn_value = 450 + nav_angle_degrees * 2.6                                   # turn_value is the desired actual_steer_value.
                turn_value = 450 + nav_angle_degrees * 3                                   # turn_value is the desired actual_steer_value.
                print("\033[1;32;40mTurn value: " + (str)(turn_value) + "\033[0;0m")

                # Machine needs to be aimed towards waypoint zero to start with.
                # This is the main steering bit:
                # There's something wrong here cos even though dot_product_angle could be +1 degree and turn_value is correct, the actual steering angle is wrong.
                
                deadzone = 6
                if (prev_GPS_X_values[5] != 0 and current_waypoint_number != 0 and self.actual_steer_angle < (turn_value +deadzone) and self.actual_steer_angle > (turn_value -deadzone) and manual_overide == False):
                    self.steerControl = 500
                    print("\033[1;35;40mNo turn currently initiated !!  \033[0;0m")
                elif (prev_GPS_X_values[5] != 0 and current_waypoint_number != 0 and self.actual_steer_angle < (turn_value -deadzone)  and manual_overide == False):     # deadzone is used to stop steering oscillating.
                    self.steerControl = 800
                    print("\033[1;35;40mTurn right initiated !!  \033[0;0m")
                elif (prev_GPS_X_values[5] != 0 and current_waypoint_number != 0 and self.actual_steer_angle > (turn_value +deadzone)  and manual_overide == False):
                    self.steerControl = 200
                    print("\033[1;35;40mTurn left initiated !!  \033[0;0m")


                # If heading buffer is not full, machine to drive straight ahead:
                if (manual_overide != True):
                    if (prev_GPS_X_values[5] == 0 and self.actual_steer_angle < 435):
                        self.steerControl = 800
                    if (prev_GPS_X_values[5] == 0 and self.actual_steer_angle > 465):
                        self.steerControl = 200
                    if (prev_GPS_X_values[5] == 0 and self.actual_steer_angle > 435 and self.actual_steer_angle < 465):
                        self.steerControl = 500    # No turn.
                if (prev_GPS_X_values[5] == 0):
                    print("\033[1;32;40mBuffer is not full, now aiming straight ahead" + "\033[0;0m")
                if (current_waypoint_number == 0):
                    print("\033[1;32;40mWay point zero has not yet been reached" + "\033[0;0m")    
                
                ## End of autonomous stuff.
                #'''
                ###############################################################################################
                
                waypoint_done = ""
                for i in range(19):
                    totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                    if(prev_waypoints[i]==1):
                        waypoint_done = "\033[1;31;40m" + " waypoint is done" + "\033[0;0m"
                    else:
                        waypoint_done = " waypoint is NOT done"
                    print("total distance to waypoint " + str(i) + ": " + (str)(round(totalDistance,0)) + waypoint_done )


                #'''
                print ("------------------------")
                    
                adc = MCP3008(channel=0) # Toggle A forwards / back
                voltage1 = 3.3 * adc.value
                # print("Toggle A forwards / back (camera gimbal): " + str(voltage1))

                adc = MCP3008(channel=1) # Toggle A left / right
                voltage2 = 3.3 * adc.value
                # print("Toggle A left / right (camera gimbal): "+ str(voltage2))

                adc = MCP3008(channel=2) # Toggle B forwards / back
                voltage3 = 3.3 * adc.value
                # print("Toggle B forwards / back: " + str(voltage3))

                adc = MCP3008(channel=3) # Toggle B left / right
                voltage4 = 3.3 * adc.value
                # print("Toggle B left / right: " + str(voltage4))

                n = n + 1
                self.pubr.publish(self.camera_pan)
                self.pubs.publish(self.camera_tilt)
                self.pubt.publish(self.throttControl)
                self.pubu.publish(self.steerControl)

                self.throttControl = ((voltage3 * 50) -60)*1.5
                #'''
                if ((voltage4 < 1.2) or (voltage4 > 1.6)):
                    manual_overide = True
                    print("\033[1;32;40mManual overide operated !! " + "\033[0;0m")
                    self.steerControl = ((voltage4 * 50) -60)*5 + 500    # This is over ridden as we are in autonomous mode.
                    print("\033[1;32;40msteerControl =  " + str(self.steerControl) + "\033[0;0m")
                else: 
                    manual_overide = False
                #'''
                # self.steerControl = ((voltage4 * 50) -60)*5 + 500
                self.camera_pan = voltage1 * 1000
                self.camera_tilt = voltage2 * 1150

                print(n," camera_pan: ",self.camera_pan)
                print(n," camera_tilt: ",self.camera_tilt)
                print(n," throttControl: ",self.throttControl)
                print(n," steerControl: ",self.steerControl)
                print ("\033[1;33;40m" + "-----------------------------------------------------------------------------------------------------------------------------" + "\033[0;0m")
                print("")
                print("")
                print("")
                print("")
                print("")
                print("")
                print("")

                self.loop_rate.sleep()

if __name__ == '__main__':
    
    
    rospy.init_node("joystick", anonymous=True)
    my_node = Nodo()
    my_node.start()
