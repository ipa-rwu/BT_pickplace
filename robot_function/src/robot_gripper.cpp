#include "robot_function/robot_gripper.h"


void GripperFunction::InitialiseGripper(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group)
{
  gripper_group = new moveit::planning_interface::MoveGroupInterface(GROUP_GRIPP);
  // gripper
  gripper_model_group = gripper_group->getCurrentState()->getJointModelGroup(GROUP_GRIPP);
  // planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

}

bool GripperFunction::OpenGripper(moveit::planning_interface::MoveGroupInterface *gripper_group, std::string target)
{
  gripper_group->setNamedTarget(target);
  gripper_group->move();
}


