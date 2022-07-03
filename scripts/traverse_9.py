#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg 
from std_msgs.msg import Int32
from math import pi
from moveit_commander.conversions import pose_to_list
#****************************************************************
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move_traverse',anonymous=True)
move_group = moveit_commander.MoveGroupCommander('manipulator')
display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
#*********************************************************************
home_pose = move_group.get_current_joint_values()
#*********************************************************************
def move_cartesian(x,y,z,scale):
  wpose=move_group.get_current_pose().pose
  wpose.position.z += z*scale  # up and down 
  wpose.position.y += y*scale  # side
  wpose.position.x += x*scale  # depth

  (plan,fraction)=move_group.compute_cartesian_path(
      [wpose], 0.01,0  # waypoints to follow  # eef_step
      )  
  move_group.execute(plan, wait=True)
  move_group.stop()
#********************************************************************
def move_rotational(joint_goal) : 
  move_group.go(joint_goal, wait=True)
  move_group.stop()
#*****************************************************************
def move_right():
  home_pose[0]-=0.05
  move_rotational(home_pose)  #move right
  move_group.stop()
#*****************************************************************
def move_left():
  home_pose[0]+=0.05
  move_rotational(home_pose)  #move left
  move_group.stop()
#*****************************************************************
def move_up():
  home_pose[2]-=0.05
  move_rotational(home_pose)
  move_group.stop()
#********************************************************************
def move_down():
  home_pose[2]+=0.05
  move_rotational(home_pose)
  move_group.stop()
#********************************************************************
def detect_aruco_14():
    pose = [-pi/2,-pi/2,pi/2,pi/2-pi/15,pi/2,-pi/2]  #detect_aruco_right
    move_rotational(pose)
#********************************************************************
def detect_aruco_10():
  pose = [0,-pi/2,pi/2,pi/2,pi/2,-pi/2]  #detect_aruco_left
  move_rotational(pose)
#********************************************************************
def detect_aruco_12():
    pose = [-1,-2*pi/3,5*pi/9+0.2,pi/9,pi/2,-pi/2]
    move_rotational(pose)
#********************************************************************
def detect_aruco_13():
    pose = [-pi/4,-pi/2+pi/8,0,pi/2+pi/9,pi/2,-pi/2]  
    move_rotational(pose)
#********************************************************************
def detect_aruco_11():
    pose = [0.35,-2*pi/3+pi/12,5*pi/9-0.35,pi/12,pi/2,-pi/2]
    move_rotational(pose)
#********************************************************************
def listener() :
    id = rospy.Subscriber('/id',Int32,callback,queue_size=1) 
    rospy.spin()
#******************************************************************
i=0
j=0
def findaruco1(id):
  global i
  if (id!=1)  :
        if(id%3==0 or id%3==2):
            move_left()
        if(id%3==1 and id!=1):
            move_up()
  if(id==1):
    i+=1
#******************************************************************
flag10=0
flag12=0
flag13=0
flag14=0
flag=0
_flag=0

def callback (id):

  global i,j,flag10,flag12,flag13,flag14,flag,_flag 
  id1=int(str(id)[-1])
  id2=int(str(id)[-2]+str(id)[-1])
  if(i<20):
    findaruco1(id)
  else:
    if(id<10 and id!=9):
      if (id == 1 or id == 2 or id == 7 or id == 8):
        move_right()
      if(id == 5 or id == 6):
        move_left()
      if(id == 3 or id ==4):
        move_down()
      if(id == 9):
        j+=1 
        if(j>10):
          _flag=1

  if (_flag==1):
        if(flag14==0):
            detect_aruco_14()
            flag14=1
        if (id2==14 and flag12==0):
            detect_aruco_12()
            flag12=1
        if (id2==12 and flag13==0):
            detect_aruco_13()
            flag13=1
        if(id2==13 and flag11==0):
            detect_aruco_11()
            flag11=1
        if (id2==11 and flag10==0):
            detect_aruco_10()
        if(id2==10 and flag==0):
            flag=1           
#*****************************************************************       
if __name__ == '__main__':
    try:
      #counter
      listener()
    except rospy.ROSInterruptException:
      pass

moveit_commander.roscpp_shutdown()
