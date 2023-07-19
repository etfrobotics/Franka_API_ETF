# Franka Emika Panda API - ETF Robotics

Custom made Panda API and opencv based computer vision package made at the University of Belgrade,
ETF Robotics laboratory.This is a lightweight, ease of use, basic manipulation API that helps student learn and develop programs for the FE Panda robot.
This tutorial is going to help you install and use our API. 

*Ubuntu version: 16.04 Xenial Xersus
*Ros version: Ros Kinetic

## Prerequisites

The control system consists of two distinct parts. The moveit control part, that needs to be compiled and installed locally on the test Bench PC; Computer vision part, that is built separately on the users PC. The first part is essential, the second part is optional and not required for basic manipulation tasks.

### Workstation:

*Ubuntu version: 16.04 LTS Xenial Xersus with a realtime kernel patch
*franka_ros 0.8.0
*frankalib 0.8.0
*Ros1 version: Ros Kinetic
*Moveit 1, for Ros Kinetic

### Remote PC

*Ubuntu version: <=22.04
*Appropriate ROS1 version compatable with the Remote PCs Ubuntu version.


## Instalation 

If you have all the prerequisites installed, skip to to the Example part of the tutorial.

### Workstation:

Libfranka requires a realtime kernel patch workstation, which can be installed using this
[tutorial](https://de3-rob1-chess.readthedocs.io/en/latest/workstation.html). After setting up the workstation with ROS kinetic using the above linked docs, you will need to install the [franka_ros](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) and [libfranka](https://frankaemika.github.io/docs/installation_linux.html) libraries.

### Remote PC:

Here is the instalation for [Ros Noetic](https://github.com/etfrobotics/AMR_tutorials).
 

## Module descriptions and general functionality documentation

### Panda API:

---
panda_api_ETF.py 

Built in point to point, cartesian movement with speed and acceleration control. 
Binary and variable grip control. General robot manipulation functionalities and server-client based communication
with the franka interface.


```
grasp_client(self,width,epsInner,epsOuter,speed,force)
```
Commands the panda effector to grip, or release an object. It has two binary states, open or closed. The user can define the speed and force of the robot grippers, as well as the expected width and grasping tolerances(epsInner, epsOuter).

```
grasp_client_variable(self,width,speed):
```

Commands the panda effector. The user can define the speed and force of the robot grippers, as well as the target width and speed of the robot grippers.

```
move_robot(self,pos,ori,dx=0,dy=0,dz=0,tol = 0.005)
```

A non-linear point-to-point movement of the robot defined by two arrays.The position is defined as an array with the x,y and z coordinates. The orientation is defined as an array with roll, pitch and yaw. With coordinate offsets(dx,dy,dz).

```
move_robot_J(self,pos,dx=0,dy=0,dz=0):
```
Moves the toolpoint to the specified position in a non-linear path.Can take a Pose, or a Coords object as a target coordinate. With respective offsets(dx,dy,dz).

```
move_robot_L_absolute(self,pos,rpy,dx=0,dy=0,dz=0)
```

A linear point-to-point movement of the robot defined by two arrays.The position is defined as an array with the x,y and z coordinates. The orientation is defined as an array with roll, pitch and yaw. With coordinate offsets(dx,dy,dz).

```
move_robot_L(self,pos,dx=0,dy=0,dz=0)
```
Moves the toolpoint to the specified position in a linear path.Can take a Pose, or a Coords object as a target coordinate. With respective offsets(dx,dy,dz).

```
set_joints(self,j1,j2,j3,j4,j5,j6,j7)
```
Sets the values of the robot joints.

```
set_vel_acc(self,value)
```
Sets the percentage value of the maxmimum speed and accelartion of the robot joints, 1 being the maximum speed the robot can produce.

```
calc_board_orietation(self)
```

Given the two object of interest features from the Yolov8 CV module, calculates the object of interest position, based on the first detected feature; the object orientation, based on the angle between the x axis of the panda emika robot and a line connecting the first and second feature of interest.


koordinate.py

Holds the trained robot coordinates and calculates relative positions to the defined anchor point.
Calculates the coordinate transformations with the estimated anchor point position and estimated board orientation.
Has aditional parsing functionalities for position memorisation.

---

### Calibration and position estimation:

---

board_orientation.py

Initial Calibration was done using the matlab calibration app. Using the obtained calibration matricies, the script unwraps the webcam image. More finite calibration is done actively with one aruco sticker used to find the workspace point because of the unrelaible mounting point of the camera, and the camera angle.

---

### Computer vision:

---

yolo_NODE.py

Implements a custom trained yolo node that sends a string of detected classes to the workstation PC. 

yolo_subscriber_node.py

Implements a workstation subscriber node that parses and sends the info to the board_orientation module.

request_server_node.py

Implements a ros service that starts the communication between the workstation PC and the remote PC. 
Use:
send a {'m','0','0'}, message to start the vision module.
---

## Examples

### 1. Say hello to Franka Emika Panda!

Panda-PC communication
Move into ws_moveit and run the following command:
```
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<the_robot_ip>
```

This command will launch all the topics and communications needed to communicate with the robot. Alternatively, use the following command to launch a simulation of the Panda Emika robot:
```
roslaunch panda_moveit_config demo.launch
``` 

### 2. Coordinate storage and memorisation

After establishing the connection to the robot, launch the moveit_commander node using:

```
rosrun moveit_commander moveit_commander_cmdline.py
```
Then using the moveit commander, type the command that will connect to the group specified after the use keyword:
```
use panda_arm
```
The command line should output a string that looks something like this:
```
joints = [0.412597410983 0.164181102811 -0.183924663199 -2.55672592441 0.0763932341404 2.7707845938 1.0086144719]
  panda_link8 pose = [
  header: 
  seq: 0
  stamp: 
    secs: 1682789626
    nsecs: 742530107
  frame_id: "/world"
  pose: 
  position: 
    x: 0.44798833408
    y: 0.0983415023025
    z: 0.21330782485
  orientation: 
    x: -0.911288604869
    y: 0.410895008805
    z: -0.0219114544515
    w: 0.0154356256343 ]
  panda_link8 RPY = [-3.0954165787477335, -0.027253823298962938, -0.7853981634]
```

The control system offers a simple coordinate memorisation system. The following snippet of code creates an instance of the Coords object, that parses out the position string of the moveit_commander:
```
from panda_api_ETF import *

# some other code ... 

Pandas_first_coordinate = Coords('''joints = [0.123968671809 0.20600802755 -0.311686978045 -2.46212787739 -0.0175176675807 2.62793402963 0.600727843634]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682793628
      nsecs:  59730052
    frame_id: "/world"
  pose: 
    position: 
      x: 0.469573183141
      y: -0.0932599124007
      z: 0.215675683591
    orientation: 
      x: 0.926509617253
      y: -0.374266160163
      z: -0.00776256462593
      w: 0.0380067505145 ]
  panda_link8 RPY = [3.0652692282630625, -0.01406146968808864, -0.767290947211288]''')
  
```  
Piece of cake! The Coords object contains the parsed out position, orientation in both quarternion and roll, pitch, yaw representation. The Coords object can also store relative position to the defined workstation coordinate system. Currently the API supports rotational and translational transformations around the vertical robot axis, so the workstation coordinate system is defined with two coordinates, aswell as offsets to the calculated coordinates in the following form:
```
from panda_api_ETF import *

# These are the coordinates relative to the global robot coordnate system:
workstation_positions = [0.5,-0.5]
workstation_orientation = math.pi/4

Pandas_second_coordinate = Coords(<string>,offsetX,offsetY,offsetZ,
				  workstation_positions,workstation_orientation)
```

### 3. Pick and place

In this tutorial you will learn how to recycle probe cables. The process is pretty straight forward, first we define a robot handler object:
```
rN = robot_dealerNODE()
```
This object contains all the necessary commands for basic manipulations. Now, using the move_robot_J(), move_robot_L(), set_vel_acc() and two coordinates of the pick up and place point, we will program the robot to recycle. 

*move_robot_J(<Coords>,offsetX,offsetY,offsetZ), moves the toolpoint to the specified coordinate in a non-linear path.
*move_robot_L(<Coords>,offsetX,offsetY,offsetZ), moves the toolpoint to the specified coordinate in a linear path.
*set_vel_acc(), sets the percentage value of the maxmimum speed and accelartion of the robot joints, 1 being the maximum speed the robot can produce.

The program should look something like this:

```
from panda_api_ETF import *

rN = robot_dealerNODE()

test_port_plug_in = Coords('''joints = [0.333021631203 0.151382270533 -0.234261829935 -2.48506457533 -0.0196069813992 2.66923524194 0.906817508729]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682791120
      nsecs: 842514038
    frame_id: "/world"
  pose: 
    position: 
      x: 0.472426941746
      y: 0.0466269562202
      z: 0.234755614071
    orientation: 
      x: 0.923091214319
      y: -0.383509347143
      z: 0.0266824238622
      w: 0.0105469880809 ]
  panda_link8 RPY = [-3.1405893997814407, -0.0573866303012271, -0.7875714377198617]''')
  
test_port_take_out = Coords('''joints = [0.338127919605 0.130485322574 -0.234318823982 -2.56966125759 -0.0193603740044 2.72324884706 0.906230387083]
  panda_link8 pose = [
  header: 
    seq: 0
    stamp: 
      secs: 1682790961
      nsecs: 242918968
    frame_id: "/world"
  pose: 
    position: 
      x: 0.450169677267
      y: 0.0450743558555
      z: 0.220324328506
    orientation: 
      x: 0.924184562993
      y: -0.381237778094
      z: 0.0208130626314
      w: 0.0103666052437 ]
  panda_link8 RPY = [3.1382905635074017, -0.046385694980998184, -0.7824272147614435]''')



#################################################################
# State1 : Probe take_out
#################################################################

rN.move_robot_J(test_port_take_out,0,0,0.05)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.set_vel_acc(0.05)
rN.move_robot_J(test_port_take_out)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.move_robot_J(test_port_take_out,0,0,0.05)

##################################################################
# State2 : Probe plugin
##################################################################
rN.move_robot_J(test_port_plug_in,0,0,0.05)
rN.set_vel_acc(0.05)
rN.move_robot_J(test_port_plug_in)
rN.move_robot_J(test_port_plug_in,0,0,-0.01)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.move_robot_J(test_port_plug_in,0,0,0.05)
rN.grasp_client(0.010,0.005,0.005,0.5,15)
rN.move_robot_L(test_port_plug_in,0,0,-0.007)
rN.set_vel_acc(0.1)
rN.move_robot_J(test_port_plug_in,0,0,0.05)
```

### 4. Robothon 2023 challenge solution.

You can take a look at a solution to the Robothon Grand Challenge competition that we developed. Its located in the robothon_task.py script. In the github repo, there is also a custom trained yolov8 model for the Robothon task, aswell as chip detection for the bring your own device part of the competition.

## Acknowledgments

We would like to express our heartfelt thanks to the ETF Robotics Research Group for their invaluable assistance. We are particularly grateful for the contributions of Assistant Nikola Knezevic, whose expertise and tireless efforts were essential to the success of our team. We would also like to extend our appreciation to Professor Kosta Jovanovic, Assistant Zavisa Gordic, and Assistant Milos Petrovic for their invaluable support and guidance. 
