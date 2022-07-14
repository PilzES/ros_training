#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface move_group_gripper("gripper");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::vector<double> home = {0.0, 0.0, 1, 57, 0.0, 0.0, 0.0};
    move_group_interface.setJointValueTarget(home);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    move_group_interface.setPlanningTime(5.0);

    move_group_interface.setPlannerId("PTP");

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_interface.execute(my_plan);
    }

    geometry_msgs::Pose capture_pose;
    capture_pose.orientation.w = 0.29;
    capture_pose.orientation.x = 0.95;
    capture_pose.orientation.y = -0.0000001;
    capture_pose.orientation.z = -0.003;
    capture_pose.position.x = -0.04;
    capture_pose.position.y = -0.56;
    capture_pose.position.z = 0.43;
    move_group_interface.setPoseTarget(capture_pose);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_interface.execute(my_plan);
    }

    std::vector<double> opened = {0.03};
    std::vector<double> closed = {0.015};

    move_group_gripper.setJointValueTarget(closed);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_g;

    move_group_gripper.setMaxVelocityScalingFactor(0.05);
    move_group_gripper.setMaxAccelerationScalingFactor(0.05);
    move_group_gripper.setPlannerId("LIN");

    success = (move_group_gripper.plan(my_plan_g) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_gripper.execute(my_plan_g);
    }

    ros::shutdown();
    return 0;
}
