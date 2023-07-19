import rospy

#!/usr/bin/env python3

import cv2
import rospy
from std_msgs.msg import String
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

#Quickfixes
#sudo rm /dev/video0
#export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python, Need to automate command

class messageNode():

    def _init_(self):
        
        rospy.init_node("Message_NODE")
        # self.pub_command = rospy.Publisher("/cmd_vel", String, queue_size = 10)
        self.sub_ = rospy.Subscriber("/yolo_pos", String, self.getInfo)
        self.classes = {}

        self.classes = {(0,0) : "blue_button",
                        (0,0) : "red_screen",
                        (0,0) : "plugn",
                        (0,0) : "slider_yellow",
                        (0,0) : "slider_green"
                        }

    def getInfo(self, data):
        s = data.data.split("|")
        self.classes["blue_button"]   = [s[0].split(",")]
        self.classes["red_screen"]    = [s[1].split(",")]
        self.classes["plugin"]        = [s[3].split(",")]
        self.classes["slider_yellow"] = [s[4].split(",")]
        self.classes["slider_green"]  = [s[5].split(",")]
