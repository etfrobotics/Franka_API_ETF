#!/usr/bin/env python3

import cv2
import rospy
from std_msgs.msg import String
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from ultralytics import YOLO

#Quickfixes
#sudo rm /dev/video0
#export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python, Need to automate command

'''
Project: Robothon competition Computer vision package
Authors: Ruzic Nikola, Rodic Filip, Biocanin Teodor
Date May, 2023
File Yolo_publisher_node.py
Contains:
    yoloNode
Description:
    Ros implementation of the yolov8 inference module
'''

class yoloNode():

    def __init__(self):
        
        rospy.init_node("Yolo_NODE")
        
        self.pub_command  = rospy.Publisher("/yolo_pos", String, queue_size = 10)
        self.sub_auto     = rospy.Subscriber("/robot_commands_yolo/command", String, self.getInfo)
        
        self.model        = YOLO('yolov8n.pt')
        self.control_mode = 'a'
        self.process_rate = 5
        self.cap          = cv2.VideoCapture(0)

        self.blue_button  = (0,0)
        self.red_screen   = (0,0)
        self.plugin       = (0,0)
        self.slider_yellow= (0,0)
        self.slider_green = (0,0)


    def getInfo(self, data):
        s = data.data.split(",")
        self.control_mode = s[0]

    def calculate_bounding_box_center(self,x1, y1, x2, y2):
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return center_x, center_y
        
    
    def auto(self):
        print("3")
        # boxes = self.results[0].boxes
        # box = boxes[0]
        # # print("1")
        # print(str(boxes))
        success, frame = self.cap.read()
        if success:
            # Run YOLOv8 inference on the frame
            results = self.model(frame)
            all_classes = results[0].boxes

            if all_classes.boxes.numel()!=0:

                all_classes = all_classes.boxes
                for el in all_classes:
                    
                    # Blue button
                    if el[5] == 0:
                        self.blue_button = self.calculate_bounding_box_center(el[0],el[1],el[2],el[3])
                        
                    # Red screen
                    if el[5] == 1:
                        self.red_screen = self.calculate_bounding_box_center(el[0],el[1],el[2],el[3])

                    # Plugin port
                    if el[5] == 2:
                        self.plugin = self.calculate_bounding_box_center(el[0],el[1],el[2],el[3])

                    # Yellow slider
                    if el[5] == 3:
                        self.slider_yellow = self.calculate_bounding_box_center(el[0],el[1],el[2],el[3])

                    # Green slider
                    if el[5] == 62:
                        self.slider_green = self.calculate_bounding_box_center(el[0],el[1],el[2],el[3])

            else:
                print("no_detections")
                self.cleanup()
                
            
            self.format_command()

    def cleanup(self):

        self.blue_button    = (0,0)
        self.red_screen     = (0,0)
        self.plugin         = (0,0)
        self.slider_yellow  = (0,0)
        self.slider_green   = (0,0)

    
    def convert2str(self,tensor_tuple):
        if tensor_tuple == (0,0):
            return str(tensor_tuple[0]) + '?' + str(tensor_tuple[1])
        else:
            return str(tensor_tuple[0].item()) + '?' + str(tensor_tuple[1].item())

    def format_command(self):
        command = ''
        command =       self.convert2str(self.blue_button)   + \
                  "|" + self.convert2str(self.red_screen)    + \
                  "|" + self.convert2str(self.plugin)        + \
                  "|" + self.convert2str(self.slider_yellow) + \
                  "|" + self.convert2str(self.slider_green) 
                
        
        self.pub_command.publish(command)

    def run(self):
        rospy.loginfo("Starting Yolo_Node")

        self.send_info()

    def send_info(self):

        r = rospy.Rate(self.process_rate)
        while not rospy.is_shutdown():
            if self.control_mode == "a":
                self.auto()
                print("nominal")
            if self.control_mode == "m":
                print("end")
                self.cap.release()
                break
            
            r.sleep()

if __name__ == '__main__':
    yN = yoloNode()
    try:
        yN.run()
    except Exception as e:
        yN.cap.release()
        print("Error: ")
        print(e)
        