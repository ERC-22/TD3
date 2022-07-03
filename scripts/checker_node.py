#!/usr/bin/env python3
from distutils.command.build_scripts import first_line_re
from multiprocessing.connection import wait
from pkgutil import ImpImporter
import sys
import rospy
import moveit_commander
import moveit_msgs.msg 
import time
import os 
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped , Pose
from std_msgs.msg import Int32, Int16, String
from math import pi, tau
from moveit_commander.conversions import pose_to_list
import copy


from erc_aruco_msg.srv import ErcArucoRequest, ErcArucoResponse, ErcAruco
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

##########################################################

def cb_1 (msg):
    # global pose_x ,pose_z ,pose_y
    pose_z = msg.pose.position.z 
    pose_x = msg.pose.position.x 
    pose_y = msg.pose.position.y

    # service_msg.tag1=[pose_x,pose_y,pose_z]
    # ex = ExampleCall()
    rospy.wait_for_service("erc_aruco_score")
        
    # create service proxy with service name and message type
    service_proxy = rospy.ServiceProxy('erc_aruco_score',ErcAruco)
    # create object of the request type for the Service (14 tags)   
    service_msg = ErcArucoRequest()

    # WARNING!#######################################################################
    #TODO: Below you should specify the tag positions with respect to the /base frame
   
    service_msg.tag1=[pose_x,pose_y,pose_z]
    service_msg.tag2=[0,0,0]
    service_msg.tag3=[0,0,0]
    service_msg.tag4=[0,0,0]
    service_msg.tag5=[0,0,0]
    service_msg.tag6=[0,0,0]
    service_msg.tag7=[0,0,0]
    service_msg.tag8=[0,0,0]
    service_msg.tag9=[0,0,0]
    service_msg.tag10=[0,0,0]
    service_msg.tag11=[0,0,0]
    service_msg.tag12=[0,0,0]
    service_msg.tag13=[0,0,0]
    service_msg.tag14=[0,0,0]

    ##################################################################################

    # call the service with your message through service proxy
    # and receive the response, which happens to be your score
    service_response = service_proxy(service_msg)
    print(f"You received score {service_response.score}")
        
    global pressed_1 


    time.sleep(2)
        
##########################################################
pressed_2 = 0
         
def cb_2 (msg2):
    
    time.sleep(2)
##########################################################

def cb_3 (msg):
    
    time.sleep(2)

##########################################################

def cb_4 (msg):
    
    
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

def cb_10 (msg):
        
    time.sleep(2)

##########################################################

def cb_11 (msg):
        
    time.sleep(2)

##########################################################

def cb_12 (msg):
        
    time.sleep(2)

##########################################################

def cb_13 (msg):
        
    time.sleep(2)

##########################################################

def cb_14 (msg):
        
    time.sleep(2)

##########################################################


rospy.Subscriber("/aruco_checker_1/pose",PoseStamped,cb_1)
rospy.Subscriber("/aruco_checker_2/pose",PoseStamped,cb_2)
rospy.Subscriber("/aruco_checker_3/pose",PoseStamped,cb_3)
rospy.Subscriber("/aruco_checker_4/pose",PoseStamped,cb_4)
rospy.Subscriber("/aruco_checker_5/pose",PoseStamped,cb_5)
rospy.Subscriber("/aruco_checker_6/pose",PoseStamped,cb_6)
rospy.Subscriber("/aruco_checker_7/pose",PoseStamped,cb_7)
rospy.Subscriber("/aruco_checker_8/pose",PoseStamped,cb_8)
rospy.Subscriber("/aruco_checker_9/pose",PoseStamped,cb_9)
rospy.Subscriber("/aruco_checker_10/pose",PoseStamped,cb_10)
rospy.Subscriber("/aruco_checker_11/pose",PoseStamped,cb_11)
rospy.Subscriber("/aruco_checker_12/pose",PoseStamped,cb_12)
rospy.Subscriber("/aruco_checker_13/pose",PoseStamped,cb_13)
rospy.Subscriber("/aruco_checker_14/pose",PoseStamped,cb_14)




# rate=rospy.Rate(100)

# rospy.spin()

