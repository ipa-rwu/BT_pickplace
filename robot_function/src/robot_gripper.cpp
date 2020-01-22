#include "robot_function/robot_gripper.h"


void GripperFunction::InitialiseGripper(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group)
{
  gripper_group = new moveit::planning_interface::MoveGroupInterface(GROUP_GRIPP);
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(gripper_group->getJointModelGroupNames().begin(), gripper_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;
  // gripper
  gripper_model_group = gripper_group->getCurrentState()->getJointModelGroup(GROUP_GRIPP);
  // planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

}

bool GripperFunction::MoveGripper(moveit::planning_interface::MoveGroupInterface *gripper_group, std::string target)
{
  std::cout << "gripper command: "<< target <<std::endl;
  gripper_group->getCurrentState();
  gripper_group->setNamedTarget(target);
  std::cout << "setNamedTarget " <<std::endl;
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // return gripper_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;;
  return (gripper_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);

}


