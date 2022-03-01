

# source /opt/ros/melodic/setup.bash && export ROS_MASTER_URI=http://master.local:11311 && cd /home/pi/ros_catkin_ws/src/joystick2 && python joystick_plus_autonomous.py

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
        prev_GPS_X_values = [0,0,0,0,0,0,0,0,0,0,0]
        prev_GPS_Y_values = [0,0,0,0,0,0,0,0,0,0,0]
        
        actual_heading = [0,0]
        norm_actual_heading = [0,0]
        desired_heading = [0,0]
        norm_desired_heading =[0,0]
        current_waypoint = [0,0]
                
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
                    for i in range (10):
                        prev_GPS_X_values[10 - i] = prev_GPS_X_values[9 - i]    # Shift the values of the array one slot to the right.
                        prev_GPS_Y_values[10 - i] = prev_GPS_Y_values[9 - i]    # Shift the values of the array one slot to the right.
                print("Array prev_GPS_X_values: " + str(prev_GPS_X_values))
                print("Array prev_GPS_Y_values: " + str(prev_GPS_Y_values))
                
                # The prev_GPS_values arrays can be used to create a heading array over (10 / rospy.Rate) seconds eg 2:
                
                actual_heading[0] = prev_GPS_X_values[0] - prev_GPS_X_values[10]
                actual_heading[1] = prev_GPS_Y_values[0] - prev_GPS_Y_values[10]
                
                
                
                
                
                
                
                
                '''
                
                # Are these above values valid? Check that prev_GPS_Y_values[10] is not equal to zero. If it is, used a valus for actual_heading from SD card.
                if (      prev_GPS_X_values[10] == 0 or prev_GPS_Y_values[10] == 0     ):
                    # df2 = read_csv('/home/pi/GPS_Way_Points/GPS_waypoints_01.csv')
                    df2 = read_csv('/home/pi/ros_catkin_ws/src/joystick2/actual_heading.csv')
                    actual_heading_data = df2.values
                    # print(actual_heading_data)
                    actual_heading[0] = actual_heading_data[0,1]
                    actual_heading[1] = actual_heading_data[0,2]
                
                
                print("Actual heading vector array: " + str(actual_heading))
                print ("------------------------")
                
                
                
                if (      prev_GPS_X_values[10] != 0 or prev_GPS_Y_values[10] != 0     ):
                    df3 = DataFrame(index=['A'], columns=['x','y','time'])
                    df3.at['A', 'x'] = actual_heading[0]
                    df3.at['A', 'y'] = actual_heading[1]
                    df3.to_csv('/home/pi/ros_catkin_ws/src/joystick2/actual_heading.csv', index=True)
                    # print df3
                
                '''
                
                
                
                
                
                
                
                
                
                
                
                
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
                
                norm_desired_heading[0] = desired_heading[0]/desired_heading_hypotenuse
                norm_desired_heading[1] = desired_heading[1]/desired_heading_hypotenuse
                
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
                
                # Determine which is the current waypoint by calculating which waypoints are behind / in front of machine:
                
                
                for i in range(20):
                    current_waypoint[0] = waypoints[i,1]
                    current_waypoint[1] = waypoints[i,2]
                    desired_heading[0] = current_waypoint[0] - self.x1
                    desired_heading[1] = current_waypoint[1] - self.y1
                
                    desired_heading_hypotenuse = math.hypot(desired_heading[0],desired_heading[1])
                
                    norm_desired_heading[0] = desired_heading[0]/desired_heading_hypotenuse
                    norm_desired_heading[1] = desired_heading[1]/desired_heading_hypotenuse
                
                    angle_rads = np.math.atan2(np.linalg.det([norm_desired_heading,norm_actual_heading]),np.dot(norm_desired_heading,norm_actual_heading))
                    nav_angle_degrees = math.degrees(angle_rads)
                    
                    if ((nav_angle_degrees < 100) and (nav_angle_degrees > -100)):
                        print("\033[1;31;40mCurrent waypoint number: " + str(i) + "\033[0;0m")
                        break
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
                
                
                
                # self.steerControl = 
                
                
                
                
                
                
                
                
                
                #'''
                
                i = 0
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 1
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 2
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 3
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 4
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 5
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 6
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 7
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 8
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 9
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 17
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))
                
                i = 18
                # print("distanceX to waypoint " + str(i) + ": " + (str)(  self.x1-waypoints[i,1]    ))
                # print("distanceY to waypoint " + str(i) + ": " + (str)(  self.y1-waypoints[i,2]    ))
                totalDistance = math.hypot((self.x1-waypoints[i,1]) , (self.y1-waypoints[i,2]))
                print("total distance to waypoint " + str(i) + ": " + (str)(   totalDistance          ))

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


                # print("")
                

                n = n + 1
                self.pubr.publish(self.camera_pan)
                self.pubs.publish(self.camera_tilt)
                self.pubt.publish(self.throttControl)
                self.pubu.publish(self.steerControl)

                self.throttControl = ((voltage3 * 50) -60)*2
                # self.steerControl = ((voltage4 * 50) -60)*5 + 500    # This is over ridden as we are in autonomous mode.
                self.camera_pan = voltage1 * 1000
                self.camera_tilt = voltage2 * 1000

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
