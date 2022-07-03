#!/usr/bin/env python3
from distutils.command.build_scripts import first_line_re
import sys
import rospy
import moveit_commander
import moveit_msgs.msg 
import time
import os 
import subprocess
from geometry_msgs.msg import PoseStamped , Pose
from std_msgs.msg import String
from math import pi, tau
from moveit_commander.conversions import pose_to_list
from tag_custom_msg_pkg.msg import tag_custom_msg

##########################################################

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('panel_auto',anonymous=True)
move_group = moveit_commander.MoveGroupCommander('manipulator')

display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
start_wpose = move_group.get_current_pose().pose  

home_pose = move_group.get_current_joint_values()
wpose=move_group.get_current_pose().pose



##########################################################

def grip_close():

    process = subprocess.Popen("rostopic pub /gripper_command std_msgs/String 'close'", shell=True,start_new_session=True)
    time.sleep(1)

    process.terminate()
    process.wait()

##########################################################

def move_rotational(joint_goal) :
    move_group.go(joint_goal, wait=True)

##########################################################

def move_cartesian(x,y,z,scale):
    time.sleep(2)
    wpose=move_group.get_current_pose().pose
    wpose.position.z += z*scale  # up and down 
    wpose.position.y += y*scale  # left
    wpose.position.x += x*scale  # forward

    (plan,fraction)=move_group.compute_cartesian_path(
      [wpose], 0.01,0) # waypoints to follow  # eef_step
    time.sleep(2)
    move_group.execute(plan, wait=True)

##########################################################

def move_right():
    move_cartesian(0,-0.15,0,1)
##########################################################
def move_left():
    move_cartesian(0,0.15,0,1)

##########################################################
def move_up():
    move_cartesian(0,0,0.1,1)

##########################################################
def move_down():
    move_cartesian(0,0,-0.18,1)

##########################################################


def first(msg) :

    wpose=move_group.get_current_pose().pose

    X = msg.pose.position.z - 0.2 - 0.017
    # - 0.2 for gripper
    # -0.17 for sw when pressed
    Y = msg.pose.position.x 
    Z = msg.pose.position.y + 0.025 + 0.03 
        
    print('first z')
    move_cartesian(0,0,Z,1)
    time.sleep(2)

    print('first y')
    move_cartesian(0,Y,0,1)
    time.sleep(2)
    
    print('first x')
    move_cartesian(X,0,0,1)
    time.sleep(2)
    ID_1.state = 1
    move_group.stop()
       
##########################################################

def second(msg2) :

    wpose=move_group.get_current_pose().pose
    
    X = msg2.pose.position.z - 0.2 - 0.017 -0.02
    Y = msg2.pose.position.x 
    Z = msg2.pose.position.y + 0.025 + 0.03

    print('second z')
    move_cartesian(0,0,Z,1)
    time.sleep(2)

    print('second y')
    move_cartesian(0,Y,0,1)
    time.sleep(2)

    print('second x')
    move_cartesian(X,0,0,1)
    time.sleep(2)


    move_group.stop()


##########################################################
def third(msg3):

    wpose=move_group.get_current_pose().pose
    
    X = msg3.pose.position.z - 0.2 - 0.017
    Y = msg3.pose.position.x 
    Z = msg3.pose.position.y + 0.025 + 0.03
 

    print('third z')
    move_cartesian(0,0,Z,1)
    time.sleep(2)

    print('third y')
    move_cartesian(0,Y,0,1)
    time.sleep(2)

    print('third x')
    move_cartesian(X,0,0,1)
    time.sleep(2)


    move_group.stop()


##########################################################
def fourth(msg4):

    wpose=move_group.get_current_pose().pose
    X = msg4.pose.position.z - 0.2 - 0.017
    Y = msg4.pose.position.x 
    Z = msg4.pose.position.y + 0.025 + 0.03

    print('fourth z')
    move_cartesian(0,0,-Z,1)
    time.sleep(2)

    print('fourth y')
    move_cartesian(0,Y,0,1)
    time.sleep(2)

    print('fourth x')
    move_cartesian(X,0,0,1)
    time.sleep(2)


    move_group.stop()


##########################################################


ID_1= tag_custom_msg()
ID_1.ID = 1
ID_1.state = 0

Switch = tag_custom_msg ()
Switch.state = 0
Switch.ID  = rospy.get_param('~Switch_ID',1)

def cb_1 (msg):

    global ID_1
    global Switch
    grip_close()
    time.sleep(3)
    
    ID_1.tag_pose = msg.pose
   
    if (ID_1.state == 0 and msg and ID_1.ID == Switch.ID):
        Switch.state = 1
        first(msg)
        print("first is enterd")
        move_cartesian(-0.02,0,0,1)
        print("move right")
        move_right()

    if (ID_1.ID != Switch.ID and Switch.state == 0):
        print("move right")
        move_right()

    if(not msg) :
        os.system("rosnode kill aruco_single_1")

    time.sleep(2)
    
    move_group.stop()
        
##########################################################

ID_2= tag_custom_msg()
ID_2.ID = 2
ID_2.state = 0
        
def cb_2 (msg2):
    
    global ID_2
    
    ID_2.tag_pose = msg2.pose

    if (ID_2.state == 0 and ID_2.ID == Switch.ID):
        Switch.state = 1
        second(msg2)
        print('second is entered')
        move_cartesian(-0.02,0,0,1)
        ID_2.state = 1
        print("move down")
        move_down()

    if (ID_2.ID != Switch.ID  and Switch.state == 0):
        print("move down")
        move_down()

    if(not msg2) :
        os.system("rosnode kill aruco_single_2")

    time.sleep(2)
    
    move_group.stop()
##########################################################

ID_3= tag_custom_msg()
ID_3.ID = 3
ID_3.state = 0

def cb_3 (msg3):
    
    pose_z = msg3.pose.position.z 
    pose_x = msg3.pose.position.x 
    pose_y = msg3.pose.position.y
    global ID_3
    
    ID_3.tag_pose = msg3.pose

    if (ID_3.state == 0 and ID_3.ID == Switch.ID):
        Switch.state = 1
        third(msg3)
        print('third is entered')
        move_cartesian(-0.02,0,0,1)
        ID_3.state = 1
    
    if (ID_3.ID != Switch.ID and Switch.state == 0):
        
        print("go home")
        time.sleep(2)
        move_rotational(home_pose)
        

    if(not msg3) :
        os.system("rosnode kill aruco_single_3")
    
    time.sleep(2)
    move_cartesian(start_wpose.position.x,start_wpose.position.x,start_wpose.position.z,1)
    
    move_group.stop()

##########################################################

ID_4= tag_custom_msg()
ID_4.ID = 4
ID_4.state = 0

def cb_4 (msg4):

    pose_z = msg4.pose.position.z 
    pose_x = msg4.pose.position.x 
    pose_y = msg4.pose.position.y
    global ID_4
    
    ID_4.tag_pose = msg4.pose

    if (ID_4.state == 0 and ID_4.ID == Switch.ID):
        Switch.state = 1
        fourth(msg4)
        print('fourth is entered')
        ID_4.state = 1
        move_cartesian(-0.02,0,0,1)
        time.sleep(2)
        print("move left")
        move_left()

    if (ID_4.ID != Switch.ID and Switch.state == 0):
        time.sleep(2)
        print("move left")
        move_left()

    if(not msg4) :
        os.system("rosnode kill aruco_single_4")

    time.sleep(2)
    
    move_group.stop()
    
    time.sleep(2)

##########################################################

def cb_5 (msg):
    
    
    time.sleep(2)

##########################################################

def cb_6 (msg):
    
    
    time.sleep(2)

##########################################################

def cb_7 (msg):
        
    time.sleep(2)

##########################################################

def cb_8 (msg):
        
    time.sleep(2)

##########################################################

def cb_9 (msg):
        
    time.sleep(2)

##########################################################


def back_home() :
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -2*pi / 3
    joint_goal[2] = 100 *pi /180
    joint_goal[3] = pi / 9
    joint_goal[4] = pi / 2
    joint_goal[5] = -pi /2
    move_group.go(joint_goal, wait=True)

##########################################################

# grip_open()
back_home()
grip_close()
move_cartesian(0.07,0,0,1)



pub_grip=rospy.Publisher("/gripper_command" , String,queue_size=1)


rospy.Subscriber("/aruco_single_1/pose",PoseStamped,cb_1,queue_size = 10)
rospy.Subscriber("/aruco_single_2/pose",PoseStamped,cb_2,queue_size = 10)
rospy.Subscriber("/aruco_single_4/pose",PoseStamped,cb_4,queue_size = 10)
rospy.Subscriber("/aruco_single_3/pose",PoseStamped,cb_3,queue_size = 10)
rospy.Subscriber("/aruco_single_5/pose",PoseStamped,cb_5,queue_size = 10)
rospy.Subscriber("/aruco_single_6/pose",PoseStamped,cb_6,queue_size = 10)
rospy.Subscriber("/aruco_single_7/pose",PoseStamped,cb_7,queue_size = 10)
rospy.Subscriber("/aruco_single_8/pose",PoseStamped,cb_8,queue_size = 10)
rospy.Subscriber("/aruco_single_9/pose",PoseStamped,cb_9,queue_size = 10)




rate=rospy.Rate(100)



rospy.spin()
