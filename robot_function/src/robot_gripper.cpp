#include "robot_function/robot_gripper.h"
#include "ur_msgs/SetIO.h"

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

void GripperFunction::SleepSafeFor(double duration)
{
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start <= ros::Duration(duration))
  {
    ros::spinOnce();
  }

}

bool GripperFunction::GripperOpen(ros::NodeHandle nh)
{
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(1);  //Pin 1 is open
  io_msg.request.state = 1;
  ros::ServiceClient client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if(client.call(io_msg))
  {
    ROS_INFO_STREAM("Open gripper initialise : " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
    SleepSafeFor(0.5);
    io_msg.request.state = 0;
    if(client.call(io_msg))
    {
      ROS_INFO_STREAM("Open gripper conclude : " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Open gripper conclude : Failed");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("Open gripper initialise : Failed");
    return false;
  }
}

bool GripperFunction::GripperClose(ros::NodeHandle nh)
{
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(0);    //Pin 0 is close
  io_msg.request.state = 1;
  ros::ServiceClient client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if(client.call(io_msg))
  {
    ROS_INFO_STREAM("Close gripper initialise :  " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
    SleepSafeFor(0.7);
    io_msg.request.state = 0;
    if(client.call(io_msg))
    {
      ROS_INFO_STREAM("Close gripper conclude :  " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Close gripper conclude : Failed");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("Close gripper initialise : Failed");
    return false;
  }
}
