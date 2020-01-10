#include "robot_function/robot_function.h"
#include "robot_function/robot_control.h"


void RobotFunction::GetBasicInfo()
{
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group->getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl;

}

void RobotFunction::InitialiseMoveit(ros::NodeHandle nh)
{
  namespace rvt = rviz_visual_tools;
  move_group = new moveit::planning_interface::MoveGroupInterface(GROUP_MANIP);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(GROUP_MANIP);

  visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();
  text_pose.translation().z() = 1.75;
  visual_tools->publishText(text_pose, "Kogrob Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

}

pathplan RobotFunction::PathPlanning(geometry_msgs::Pose target_pose)
{
    namespace rvt = rviz_visual_tools;
    pathplan result;
    std::cout<<target_pose<<std::endl;
    move_group->setPoseTarget(target_pose);

    // show 
    visual_tools->publishAxisLabeled(target_pose, "pose");
    visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools->publishTrajectoryLine(result.plan.trajectory_, joint_model_group);
    visual_tools->trigger();
    std::cout<<"here1"<<std::endl;
    result.success = (move_group->plan(result.plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout<<"here2"<<std::endl;
    ROS_INFO_NAMED("Demo", "Visualizing plan (pose goal) %s", result.success ? "" : "FAILED");
    return result;
}

bool RobotFunction::MoveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan plan)
{
  move_group->setStartStateToCurrentState();
  return move_group->execute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

// bool PlanExecute(geometry_msgs::Pose target_pose)
// {
//       namespace rvt = rviz_visual_tools;
//     pathplan result;
//     std::cout<<target_pose<<std::endl;
//     move_group->setPoseTarget(target_pose);

//     // show 
//     visual_tools->publishAxisLabeled(target_pose, "pose");
//     visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//     visual_tools->publishTrajectoryLine(result.plan.trajectory_, joint_model_group);
//     visual_tools->trigger();
//     std::cout<<"here1"<<std::endl;
//     result.success = (move_group->plan(result.plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     std::cout<<"here2"<<std::endl;
//     ROS_INFO_NAMED("Demo", "Visualizing plan (pose goal) %s", result.success ? "" : "FAILED");
//     move_group->setStartStateToCurrentState();
//     return move_group->execute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
// }