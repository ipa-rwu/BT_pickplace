#include "robot_function/robot_function.h"


RobotFunction::RobotFunction(ros::NodeHandle nh)
{
    camera_subscriber = nh.subscribe<geometry_msgs::Pose>("/camera/target/pose", 1000, &RobotFunction::CameraCallback, this);
    holdobj_subscriber = nh.subscribe<std_msgs::Bool>("/camera/target/hold", 1000, &RobotFunction::HoldObjCallback, this);
}

RobotFunction::~RobotFunction()  {} 


void RobotFunction::GetBasicInfo(moveit::planning_interface::MoveGroupInterface *move_group)
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


void RobotFunction::InitialiseMoveit(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
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
  pub_fake_hold_obj = nh.advertise<std_msgs::Bool>("/camera/target/hold", 1);
  // planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

}


// true same pose
bool RobotFunction::comparePoses(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::Pose pose2, double delta_posistion, double delta_orientation)
{
  geometry_msgs::Pose pose1 = move_group->getCurrentPose().pose;
  if (  abs(pose1.position.z-pose2.position.z ) <= delta_posistion
        && abs(pose1.position.y-pose2.position.y ) <= delta_posistion
        && abs(pose1.position.z-pose2.position.z ) <= delta_posistion
        && abs(pose1.orientation.x - pose2.orientation.x) <= delta_orientation
        && abs(pose1.orientation.y - pose2.orientation.y) <= delta_orientation
        && abs(pose1.orientation.z - pose2.orientation.z) <= delta_orientation
        && abs(pose1.orientation.w - pose2.orientation.w) <= delta_orientation
     )
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RobotFunction::CameraCallback(const geometry_msgs::Pose::ConstPtr& camera_msg)
{
  // msg: {"data": "start"}
  newTarget.position = camera_msg->position;
  newTarget.orientation = camera_msg->orientation;
  ROS_INFO_STREAM("Camera callback heard position:" << newTarget.position.y);
  TagGetTargetPose = true;
}

void RobotFunction::HoldObjCallback(const std_msgs::Bool::ConstPtr& holdobj_msg)
{
  if (holdobj_msg->data)
  {
      TagHoldObj =true;
  }
  else
  {
      TagHoldObj = false;
  }
  ROS_INFO_STREAM("HoldObjCallback callback heard TagHoldObj: " << holdobj_msg->data);
}

bool RobotFunction::IsHoldObj()
{
  if(TagHoldObj == true)
  {
    std::cout << "IsHoldObj: yes" << std::endl;
    return true;
  }
  else if(TagHoldObj == false)
  {
    // std::cout << "IsHoldObj: no" << std::endl;
    return false;
  }
  
}

// Planning to a Pose goal or taget name
pathplan RobotFunction::PathPlanning(geometry_msgs::Pose target_pose, std::string target_name, moveit::planning_interface::MoveGroupInterface *move_group)
{
    // TagGetTargetPose = false;
    std::cout<<"pathpanning"<<std::endl;

    pathplan result;
    move_group->setStartState(*move_group->getCurrentState());

    // provide target pose
    if (target_name == "none")
    {
      std::cout<<"target:" << target_pose << std::endl;
      move_group->setPoseTarget(target_pose);
    }
    // provide target name
    else
    {
      move_group->setNamedTarget(target_name);
    }
    result.success = (move_group->plan(result.plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return result;
}


void RobotFunction::AdjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory)
{

  std::vector<ros::Duration> times_from_start;
  times_from_start.resize(trajectory.joint_trajectory.points.size());

  for(int i=0; i < times_from_start.size() ; i++)
  {
    times_from_start[i]= trajectory.joint_trajectory.points[i].time_from_start;
  }

  bool adjusted_flag=false;
  for(int i=1; i< times_from_start.size()-1;i++)
  {
    if(times_from_start[i]==ros::Duration(0))
    {
      ros::Duration prev = times_from_start[i];
      times_from_start[i] = ros::Duration((times_from_start[i-1].toSec()+times_from_start[i+1].toSec())/2.0);
      // ROS_WARN_STREAM("Recomputing point " << i << " from " << prev <<  " to: " << times_from_start[i-1] << " + " << times_from_start[i+1] << " = " <<times_from_start[i]);
      adjusted_flag=true;
    }
  }

  if( times_from_start.size()>1 &&  times_from_start[times_from_start.size()-1] == ros::Duration(0))
  {
    ROS_WARN_STREAM("Final point in trajectory has 0 timestamp, incrementing logically");
    times_from_start[times_from_start.size()-1] = times_from_start[times_from_start.size()-2] + ros::Duration(0.1);
    adjusted_flag=true;
  }

  if(adjusted_flag)
  {
    for(int i=0; i< times_from_start.size(); i++)
    {
      trajectory.joint_trajectory.points[i].time_from_start = times_from_start[i];
      // ROS_INFO_STREAM("Recomputed time point " << i << " : " << trajectory.joint_trajectory.points[i].time_from_start );
    }
  }

}


// Planning Cartesian Paths waypoints
pathplan RobotFunction::CartesianPathPlan(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface *move_group,
double eef_step, double jump_threshold)
{
  // TagGetTargetPose = false;
  pathplan result;
  move_group->setStartState(*move_group->getCurrentState());
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(move_group->getCurrentPose().pose);
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  double fraction=0.0;
  while(fraction<0.5)
  {
    fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_STREAM("fraction: " << fraction );

  }

  AdjustTrajectoryToFixTimeSequencing(trajectory);

  result.plan.trajectory_ = trajectory;
  result.success = true;

  return result;
}




/*
bool RobotFunction::PathPlanning(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface *move_group)
{
    std::cout<<"pathpanning"<<std::endl;
    namespace rvt = rviz_visual_tools;
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.15;
    target_pose.orientation.w = 1.0;
    std::cout<<"target:" << target_pose << std::endl;
    move_group->setPoseTarget(target_pose);
    std::cout<<"set target" <<  std::endl;
    // show 
    // visual_tools->publishAxisLabeled(target_pose, "pose");
    // visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools->publishTrajectoryLine(plan.trajectory_, joint_model_group);
    // visual_tools->trigger();
    std::cout<<"move" <<  std::endl;
    success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Demo", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
    return success;
}
*/

// bool RobotFunction::MoveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan plan)
// {
//   move_group->setStartStateToCurrentState();
//   TagGetTargetPose = false;
//   return move_group->execute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
// }

bool RobotFunction::MoveGripper(moveit::planning_interface::MoveGroupInterface *move_group, std::string target)
{
  std::cout << "gripper command: "<< target <<std::endl;
  move_group->getCurrentState();
  move_group->setNamedTarget("open");
  std::cout << "setNamedTarget " <<std::endl;
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // return gripper_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;;
  return (move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);

}


gettarget RobotFunction::CameraFindTarget()
{
  gettarget newtarget;
  newtarget.success = TagGetTargetPose;
  TagGetTargetPose = false;
  //  std::cout << "TagGetTargetPose: "<< TagGetTargetPose << std::endl;

  if (newtarget.success)
  {
    newtarget.target_pose = newTarget;
    std::cout << "CameraFindTarget: "<< newtarget.success << std::endl;
    return newtarget;
  }
}

gettarget RobotFunction::KeepDistanceToTarget(geometry_msgs::Pose target_pose, double height)
{
  gettarget newtarget;
  newtarget.target_pose.position.x = target_pose.position.x;
  newtarget.target_pose.position.y = target_pose.position.y;
  newtarget.target_pose.position.z = target_pose.position.z + height;
  newtarget.target_pose.orientation.x = target_pose.orientation.x;
  newtarget.target_pose.orientation.y = target_pose.orientation.y;
  newtarget.target_pose.orientation.z = target_pose.orientation.z;
  newtarget.target_pose.orientation.w = target_pose.orientation.w;
  newtarget.success = true;
  return newtarget;
}



bool RobotFunction::MoveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{
  move_group->setStartStateToCurrentState();
  return move_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
    // bool success = (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // move_group->move();
}

// // robotiq gripper
// bool RobotFunction::OpenGripper()
// {

// }

void RobotFunction::PubFakeHoldObj()
{
  std_msgs::Bool msg;
  msg.data = true;
  pub_fake_hold_obj.publish(msg);
}