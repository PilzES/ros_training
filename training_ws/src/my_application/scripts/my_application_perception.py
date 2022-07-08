#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point, Quaternion
from pilz_robot_programming import Robot, Gripper, from_euler, Lin, Ptp
from my_perception.srv import *

import math
import rospy
import sys
import os

from std_msgs.msg import String
from sensor_msgs.msg import JointState


__REQUIRED_API_VERSION__ = "1"    # API version
__ROBOT_VELOCITY__ = 0.3   # velocity of the robot

def locatePart():
    try:
        partUpdater = rospy.ServiceProxy('partUpdate', partUpdate)
        resp = partUpdater()
        rospy.loginfo("part updated") # log
        return resp.updated

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def start_program():
  
  capture_pose = Pose(
	  position = Point(x = -0.451079682469,y = -0.0142757191132,z = 0.453848135043),
    orientation = from_euler(0,-1.57075,0))

  aprox_pick_pose = Pose(position=Point (0, 0, -0.10), orientation=from_euler(0, math.radians(0), math.radians(0))) # cartesian coordinates
  pick_pose = Pose(position=Point (0, 0, 0.022), orientation=from_euler(0, math.radians(0), math.radians(0))) # cartesian coordinates

  aprox_pose_2 = Pose(
    position = Point(x = 0.03985, y = -0.410136, z = 0.220053),
    orientation = Quaternion(x = -0.00059,y = 0.960858,z = 0.00438,w = 0.277002))

  throw_cube = Pose(
    position = Point(x = 0.08697,y = -0.409354,z = 0.145142),
    orientation = Quaternion(x = -0.00059,y = 0.960858,z = 0.00438,w = 0.277002))
  
  while True:
    r.move(Gripper(0.03, vel_scale = __ROBOT_VELOCITY__))
    rospy.loginfo("Moving to get 3D capture position") # log
    r.move(Ptp(goal=capture_pose, vel_scale = __ROBOT_VELOCITY__, relative=False))
    rospy.loginfo("At capture_pose") # log
    rospy.sleep(0.2) # Sleeps for 0.2 sec

    locatePart()

    r.move(Ptp(goal=aprox_pick_pose, vel_scale =__ROBOT_VELOCITY__, relative=False, reference_frame="cluster_0"))
    rospy.loginfo("At aprox_capture_pose, Z=+0.1m ") # log
    r.move(Lin(goal=pick_pose, vel_scale = 0.05, relative=False, reference_frame="cluster_0"))
    r.move(Gripper(0.021492, vel_scale = __ROBOT_VELOCITY__))
    r.move(Lin(goal=aprox_pick_pose, vel_scale = 0.05, relative=False, reference_frame="cluster_0"))
    r.move(Ptp(goal=aprox_pose_2, vel_scale = __ROBOT_VELOCITY__, relative=False))
    r.move(Lin(goal=throw_cube, vel_scale = 0.05, acc_scale= 0.1, relative=False))
    r.move(Gripper(0.03, vel_scale = __ROBOT_VELOCITY__))
    r.move(Lin(goal=aprox_pose_2, vel_scale = __ROBOT_VELOCITY__, relative=False))
  

if __name__ == "__main__":

    # Create a rosnode
    rospy.init_node('robot_program_node')
    # Create the robot 
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    rospy.wait_for_service('partUpdate')

    # start the main program
    start_program()
    rospy.spin()
    

  
