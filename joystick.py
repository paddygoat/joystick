
# source /opt/ros/melodic/setup.bash
# source /home/pi/ros_catkin_ws/devel_isolated/setup.bash
# cd /home/pi/ros_catkin_ws/src/joystick && python joystick.py

# https://stackoverflow.com/questions/37373211/update-the-global-variable-in-rospy
# export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2


import rospy
import std_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

import os
import time
from datetime import datetime

from gpiozero import MCP3008

class Nodo(object):
    def __init__(self):

        self.throttControl = 67.0
        self.steerControl = 49.0
        self.loop_rate = rospy.Rate(5)
        self.pubt = rospy.Publisher("throttControl_msg", Float32, queue_size=10)
        self.pubu = rospy.Publisher("steerControl_msg", Float32, queue_size=10)

    def start(self):
        n = 0
        while not rospy.is_shutdown():

            while(True):
                adc = MCP3008(channel=0) # Toggle A forwards / back
                voltage1 = 3.3 * adc.value
                print("Toggle A forwards / back: ",voltage1)

                adc = MCP3008(channel=1) # Toggle A left / right
                voltage2 = 3.3 * adc.value
                print("Toggle A left / right: ",voltage2)

                adc = MCP3008(channel=2) # Toggle B forwards / back
                voltage3 = 3.3 * adc.value
                print("Toggle B forwards / back: ",voltage3)

                adc = MCP3008(channel=3) # Toggle B left / right
                voltage4 = 3.3 * adc.value
                print("Toggle B left / right: ",voltage4)

                print("")

                n = n + 1
                self.pubt.publish(self.throttControl)
                self.pubu.publish(self.steerControl)

                self.throttControl = ((voltage3 * 50) -60)*2
                self.steerControl = ((voltage4 * 50) -60)*5 + 500

                print(n," throttControl",self.throttControl)
                print(n," steerControl",self.steerControl)
                print("")

                self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("joystick", anonymous=True)
    my_node = Nodo()
    my_node.start()
