#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Robot, Gripper, from_euler, Lin, Ptp
import math
import rospy
import sys
# Moveit imports 
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

import threading

__REQUIRED_API_VERSION__ = "1"    # API version
__ROBOT_VELOCITY__ = 0.7     # velocity of the robot


# main program
def start_program():
    
    global scene
    global robot
    scene.remove_world_object("relay")
    rospy.loginfo("Program started") # log
    hmi_publisher.publish("Starting Program")
    # important positions
    start_pos = [1.49, -0.54, 1.09, 0.05, 0.91,-1.67]   # joint values

    pick_pose = Pose(position=Point (0, -0.525, 0.25), orientation=from_euler(0, math.radians(180), math.radians(-45))) # cartesian coordinates
    work_station_pose = Pose(position=Point(-0.5, 0.1, 0.2) , orientation=from_euler(0, math.radians(-135), math.radians(90)))  # cartesian coordinates
    place_pose = Pose(position=Point(-0.1,0.4,0.25) , orientation=from_euler(0, math.radians(180),  math.radians(90))) # cartesian coordinates
    
  
    # move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=start_pos, vel_scale=__ROBOT_VELOCITY__))

    rospy.loginfo("Start loop") # log

    while(True):
        # do infinite loop
        scene.remove_world_object("relay")
        add_box()

        # pick the PNOZ
        hmi_publisher.publish("Move to Pick Position")
        rospy.loginfo("Move to pick position") # log
        r.move(Ptp(goal=pick_pose, vel_scale = __ROBOT_VELOCITY__, relative=False))
        rospy.loginfo("Pick movement") # log
        pick_and_place()
        
        # put the PNOZ in a "machine"
        hmi_publisher.publish("Move to virtual machine")
        rospy.loginfo("Move to virtual machine") # log
        r.move(Ptp(goal=work_station_pose,vel_scale = __ROBOT_VELOCITY__, relative=False, acc_scale=0.5))
        hmi_publisher.publish("Place PNOZ in machine")
        rospy.loginfo("Place PNOZ in machine") # log
        pick_and_place()
        rospy.sleep(1)      # Sleeps for 1 sec (wait until work is finished)
        hmi_publisher.publish("Pick PNOZ from machine")
        rospy.loginfo("Pick PNOZ from machine") # log
        pick_and_place()

        # place the PNOZ
        hmi_publisher.publish("Move to place position")
        rospy.loginfo("Move to place position") # log
        r.move(Ptp(goal=place_pose, vel_scale = __ROBOT_VELOCITY__, relative=False))
        rospy.loginfo("Place movement") # log
        pick_and_place()
    
def toggleGripper(): 

    global isPicking

    if isPicking: 
        r.move(Gripper(0.03, vel_scale = __ROBOT_VELOCITY__))

        try: 
            scene.remove_attached_object("prbt_tcp", name="relay")

        except: 

            pass
    else: 

        r.move(Gripper(0.015, vel_scale = __ROBOT_VELOCITY__))
        #raw_input("about to attach box...")

        try: 
            scene.attach_box("prbt_tcp", "relay", touch_links=['prbt_gripper_finger_left_link, prbt_gripper_finger_right_link'])
            
        except: 

            pass

def pick_and_place():
    """pick and place function"""
    toggleGripper()
    # a static velocity of 0.2 is used
    # the position is given relative to the TCP.
    r.move(Lin(goal=Pose(position=Point(0, 0, 0.1)), reference_frame="prbt_tcp", vel_scale=0.7, acc_scale=0.2))
    rospy.loginfo("Open/Close the gripper") # log
    global isPicking 

    isPicking = not isPicking
    toggleGripper()
    rospy.sleep(0.2)    # pick or Place the PNOZ (close or open the gripper)
    r.move(Lin(goal=Pose(position=Point(0, 0, -0.1)), reference_frame="prbt_tcp", vel_scale=0.7, acc_scale=0.2))

def moveit_setup(): 
    global scene 
    global robot 
    global move_group 

    # Moveit Setup 
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)
    print(dir(scene))
    print(dir(robot))
    # Remove object if it was present previously 
    scene.remove_world_object("relay")

    # group_name = "manipulator"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    # eef_link = move_group.get_end_effector_link()
    # print(dir(move_group))
    

    #print(robot.get_link_names(group='gripper'))

def add_box():
    global scene 
    # Add relay with selected dimensions
    dimensions = [0.1, 0.1, 0.01]
    # Stop function if array passed is not the expected size 
    if not len(dimensions) == 3: 
        
        rospy.loginfo("Dimensions array passed is not the expected size (should be 3, is %s)" % str(len(dimensions))) 
        return 
    else: 

        print("Dimensions Ok")

    width = dimensions[0]
    depth = dimensions[1]
    height = dimensions[2]

    # Remove object if it was present previously 
    #scene.remove_world_object("relay")

    # Instantiate and configure pose 
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "prbt_base_link"
    box_pose.pose.orientation.w = 1.0
    box_name = "relay"
    box_pose.pose.position = Point(0, -0.525, 0.05)
    # Add the box via the moveit interface
    scene.add_box(box_name, box_pose, size=(width, height, depth))
    #raw_input("just added box...")

def hmi_button_topic(msg): 

    print("Reciever HMI command: ->" + msg.data)

    if msg.data == "start":

        r.resume()
        hmi_publisher.publish("Starting")

    elif msg.data == "pause": 

        r.pause()
        hmi_publisher.publish("Paused")


if __name__ == "__main__":

    # Create a rosnode
    rospy.init_node('robot_program_node')

    # Create the robot 
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # Variable for controlling pick of object
    global isPicking
    isPicking = True

    # Setup moveit for picking object
    moveit_setup()

    # Subscriber for hmi buttons
    rospy.Subscriber("hmi_buttons", String, hmi_button_topic)

    # Publisher for hmi status
    hmi_publisher = rospy.Publisher("hmi_status", String, queue_size=1)

    # start the main program
    start_program()
