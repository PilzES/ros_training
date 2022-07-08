#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def go_to_pose_goal(rob):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = -0.451079682469
        pose_goal.position.y = -0.0142757191132
        pose_goal.position.z = 0.453848135043
        pose_goal.orientation.x = 0.709177308702
        pose_goal.orientation.y = 0.0308413740642
        pose_goal.orientation.z = -0.7038934019
        pose_goal.orientation.w = 0.0255035924904 
        
        rob.set_pose_target(pose_goal)
        plan = rob.go(wait=True)
        rob.stop()
        rob.clear_pose_targets()
        current_pose = rob.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

def init():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_robot = "manipulator"
    move_group_robot = moveit_commander.MoveGroupCommander(group_robot)

    group_gripper = "gripper"
    move_group_gripper = moveit_commander.MoveGroupCommander(group_gripper)

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,)

    planning_frame = move_group_robot.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    eef_link = move_group_gripper.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    return move_group_robot, move_group_gripper

if __name__=="__main__":
    rob,grip=init()

    go_to_pose_goal(rob)

