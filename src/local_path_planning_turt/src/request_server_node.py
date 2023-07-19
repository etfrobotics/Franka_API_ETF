#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

from local_path_planning_turt.srv import porukica, porukicaResponse

# ROS service use cheatsheet:
########################################################################

#rosservice call /<name_service> + 2 x tab, to format the message
#rosservice call /man + 2 x tab

###########################################################################

class server():

    def __init__(self):

        rospy.init_node("server")
        self.pub_command = rospy.Publisher("/robot_commands_yolo/command", String, queue_size=5)
        

        
        #ideja, tri publisha, dva za vrednosti, 

    def callback(self,req):

        # Callback function for the rospy "man" service.

        # To access the neccessary data, the rospy.msg fields are called. 

        mode = req.mode
        firstarg = req.b
        secondarg = req.c

        if mode == 'a':
            print("automatski rezim")
            self.pub_command.publish(mode + ',' + firstarg + ',' + secondarg)
            

        elif mode == 'm':
            print("manuelni rezim")
            # if firstarg == 'f' or firstarg == 'b' or firstarg == 's' or firstarg == 'ccw' or firstarg =='cw':
            self.pub_command.publish(mode + ',' + firstarg + ',' + 'l')
            # else:
            # print('Invallid command for manual control')
            
        else:
            return porukicaResponse("Invalid commands")
            

        #returns True response to the client if the write is successful.
        return porukicaResponse("Command has been sent")

    def main(self):
        #Main function that calls all the need helper function for the service to opperate.
        service = rospy.Service("controller_service", porukica, self.callback)
        rospy.loginfo("Service is ready!")
        rospy.spin()

if __name__ == '__main__':
    s = server()
    s.main()