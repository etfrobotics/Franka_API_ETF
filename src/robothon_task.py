import panda_api_ETF
from panda_api_ETF import *
from board_orientation import *
from slider_detect import *
from koordinate import *

rN = robot_dealerNODE()
board_origin = rN.classes["blue_button"]
board_orientation = rN.calc_board_orietation()
# gripper needs to be in an opened state
# todo: add a check if the gripper is in an opened state
# rN.grasp_client(0.010,0.005,0.005,0.5,15)

################################################################
# State0 : take a pic
################################################################

pos= [0.5, -0.1, 0.7]
ori= [math.pi,0, -math.pi/4]
rN.move_robot(pos,ori)
rN.set_vel_acc(0.1)



# board_origin, board_orientation,_ = vision()

# print("gledaj vamo")
# print(board_origin, board_orientation)

# board_origin = [0.479786920544, 0.00132797830077]
# board_orientation = 0
# cv2.destroyAllWindows()

# board_origin = [0.434919172465, 0.00149003666781] 

start_button_press, slider_snapshot, slider_push, slider_push_back, test_port_take_out, test_port_plug_in, hatch_grasped, hatch_lift_15, hatch_lift_30, hatch_lift_60, hatch_lift_75, hatch_lift_90, hatch_lift_100, hatch_lift_depart, cable_grab, cable_take_out, cable_plug_in_approach, cable_plug_in_prepare, end_button_press = calculate_coordinates(board_origin, board_orientation)

print(start_button_press.pos_board_cs)

################################################################
# State1 : button press
################################################################
rN.move_robot_J(start_button_press,0,0,0.05)
rN.set_vel_acc(0.05)
rN.move_robot_J(start_button_press)
rN.move_robot_J(start_button_press,0,0,0.05)
rN.set_vel_acc(0.1)
        
################################################################
# State2 : slider push
################################################################

rN.move_robot_J(slider_snapshot)
###################################

# slider_percentage_1 = get_yellow_perc()/100+0.04
slider_percentage_1 = 0.53
###################################
rN.set_vel_acc(0.05)
rN.move_robot_L(slider_push,0,0,0.05)
rN.move_robot_J(slider_push)


slider_push_end = Coords('''joints = [0.334066071347 0.383714202111 -0.21081906773 -2.1869625966 0.056998641396 2.60584245019 -0.652971490034]
  panda_link8 pose = [
  header: 
  seq: 0
  stamp: 
    secs: 1682790182
    nsecs: 909615993
  frame_id: "/world"
  pose: 
  position: 
    x: 0.530357284134
    y: 0.0675537744405
    z: 0.217556904856
  orientation: 
    x: 0.931013562477
    y: 0.363499840159
    z: 0.0154826000016
    w: 0.0290155438607 ]
  panda_link8 RPY = [3.0762604511881086, -0.007740819463268916, 0.7447027157679146]''',1,1,slider_percentage_1,board_origin,board_orientation)

rN.move_robot_J(slider_push_end)
rN.move_robot_J(slider_push)
rN.move_robot_J(slider_snapshot)
########################################

# slider_percentage_2 = get_green_perc()/100+0.04
slider_percentage_2 = 0.22
########################################

# rN.move_robot_J(slider_switch_side)

if (slider_percentage_1 > slider_percentage_2):
    rN.move_robot_J(slider_push_back,0,0,0.05)
    rN.move_robot_J(slider_push_back)


    slider_push_back_end = Coords('''joints = [0.365462654269 0.250319810029 -0.219253089769 -2.42041105082 0.0332876862321 2.70343168099 -0.655756284205]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682790690
      nsecs: 475886106
    frame_id: "/world"
  pose: 
    position: 
      x: 0.523420125416
      y: 0.0682959461525
      z: 0.212955051957
    orientation: 
      x: 0.924597312242
      y: 0.379935502414
      z: 0.0141269232975
      w: 0.0238590494018 ]
  panda_link8 RPY = [3.086705425315451, -0.00800149170423337, 0.7799933119358475]''',1,0,slider_percentage_2,board_origin,board_orientation)

    rN.move_robot_J(slider_push_back_end)

else:
    rN.move_robot_J(slider_push,0,0,0.05)
    rN.move_robot_J(slider_push)

    slider_push_end = Coords('''joints = [0.334066071347 0.383714202111 -0.21081906773 -2.1869625966 0.056998641396 2.60584245019 -0.652971490034]
    panda_link8 pose = [
    header: 
    seq: 0
    stamp: 
        secs: 1682790182
        nsecs: 909615993
    frame_id: "/world"
    pose: 
    position: 
        x: 0.5357284134
        y: 0.0675537744405
        z: 0.217556904856
    orientation: 
        x: 0.931013562477
        y: 0.363499840159
        z: 0.0154826000016
        w: 0.0290155438607 ]
    panda_link8 RPY = [3.0762604511881086, -0.007740819463268916, 0.7447027157679146]''',1,1,slider_percentage_2,board_origin,board_orientation)
    

    rN.move_robot_J(slider_push_end)

        
##################################################################
# State3 : Probe takeout 
##################################################################
rN.set_joints(0.331871547138, -0.659965603918, -0.240024021077, -2.71483051872, -0.184904711679, 2.0982702793, 1.01525540689)
rN.move_robot_J(test_port_take_out,0,0,0.05)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.set_vel_acc(0.05)
rN.move_robot_J(test_port_take_out)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.move_robot_J(test_port_take_out,0,0,0.05)

##################################################################
# State4 : Probe plugin
##################################################################
rN.move_robot_J(test_port_plug_in,0,0,0.05)
rN.set_vel_acc(0.05)
rN.move_robot_J(test_port_plug_in)
rN.move_robot_J(test_port_plug_in,0,0,-0.01)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.move_robot_J(test_port_plug_in,0,0,0.05)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.move_robot_L(test_port_plug_in,0,0,-0.003)
rN.move_robot_J(test_port_plug_in,0,0,0.05)
rN.move_robot_L(test_port_plug_in,0,0,-0.003)
rN.move_robot_J(test_port_plug_in,0,0,0.05)
rN.move_robot_L(test_port_plug_in,0,0,-0.003)
rN.set_vel_acc(0.1)
rN.move_robot_J(test_port_plug_in,0,0,0.05)

##################################################################
# State5 : Hatch opening
##################################################################


rN.grasp_client_variable(0.03,0.1)
rN.move_robot_J(hatch_grasped,0,0,0.05)
rN.move_robot_J(hatch_grasped)
rN.grasp_client(0.015,0.005,0.005,0.5,5)
rN.move_robot_J(hatch_lift_15)
rN.move_robot_J(hatch_lift_30)
rN.move_robot_J(hatch_lift_60)
rN.move_robot_J(hatch_lift_75)
rN.move_robot_J(hatch_lift_90)
rN.move_robot_J(hatch_lift_100)
rN.grasp_client(0.015,0.005,0.005,0.5,5)
rN.move_robot_J(hatch_lift_depart)


rN.set_joints(0.331871547138, -0.659965603918, -0.240024021077, -2.71483051872, -0.184904711679, 2.0982702793, 1.01525540689)

##################################################################
# State6 : Cable
##################################################################

rN.move_robot_J(cable_grab,0,0,0.2)
rN.move_robot_J(cable_grab,0,0,0.002)
rN.grasp_client(0.007,0.002,0.002,0.5,5)
rN.move_robot_J(cable_take_out)
rN.move_robot_J(cable_take_out,0,0,0.2)
rN.move_robot_J(cable_plug_in_approach)
rN.move_robot_J(cable_plug_in_prepare,0,0,0.010)
# rN.move_robot_J(cable_plug_in_prepare)
# rN.grasp_client_variable(0.009,0.1)
rN.move_robot_J(cable_plug_in_approach)
rN.move_robot_J(cable_take_out,0,0,0.2)
rN.move_robot_J(cable_take_out)
rN.move_robot_J(cable_grab,0,0,0.002)
rN.grasp_client(0.015,0.005,0.005,0.5,5)
rN.move_robot_J(cable_grab,0,0,0.01)


#################################################################
# State7 : Cable wrap 
##################################################################

# Todo: Using a loose grip, wrap the cable around the spokes, making a bigger wrap
# angle when needed to keep the cable tension

#################################################################
# State8 : End button
#################################################################

rN.grasp_client(0.015,0.005,0.005,0.5,5)
rN.move_robot_J(end_button_press,0,0,0.05)
rN.move_robot_J(end_button_press)

# Reset
##################################################################
# rN.set_joints(0.331871547138, -0.659965603918, -0.240024021077, -2.71483051872, -0.184904711679, 2.0982702793, 1.01525540689)
rN.grasp_client(0.010,0.005,0.005,0.5,15)