#include "robot_function/bt.h"

//camera
// BT::NodeStatus BTWaitForTarget::tick()
// {
//   RobotFunction robot_obj(_nh);
//   gettarget subtarget = robot_obj.CameraFindTarget();
//   std::cout << "BTWaitForTarget: "<< subtarget.success << std::endl;
  
//   if(_success = subtarget.success)
//     {
//     std::cout << "BTWaitForTarget: SUCCESS"<< std::endl;
//     setOutput<geometry_msgs::Pose>("target", subtarget.target_pose);
//     return BT::NodeStatus::SUCCESS;
//     }
//   return BT::NodeStatus::FAILURE;  
    
// }

// BT::NodeStatus BTWaitForTarget::tick()
// {
//   RobotFunction robot_obj(_nh);
//   bool subtarget = robot_obj.CameraFindTarget();
//   std::cout << "BTWaitForTarget: "<<subtarget<< std::endl;
  
//   if(_success = subtarget)
//     {
//     std::cout << "BTWaitForTarget: SUCCESS"<< std::endl;
//     // setOutput<geometry_msgs::Pose>("target", subtarget.target_pose);
//     return BT::NodeStatus::SUCCESS;
//     }
//   return BT::NodeStatus::FAILURE;  
    
// }

/*
BT::NodeStatus BTWaitForTarget::tick()
{
  RobotFunction robot_obj(_nh);
    counter++;
    std::cout<<"print "<< counter <<std::endl;
    _halted = false;  
    gettarget subtarget;
    subtarget.success = false;
    while (!subtarget.success)
    {
      subtarget = robot_obj.CameraFindTarget();

      if(!subtarget.success)
      {
        setStatusRunningAndYield();

      }

    }
    std::cout << "FindTarget: SUCCESS"<< std::endl;
    setOutput<geometry_msgs::Pose>("target", subtarget.target_pose);

    return BT::NodeStatus::SUCCESS;
}

// BT::PortsList BTWaitForTarget::providedPorts()
// {
//   return { BT::OutputPort<geometry_msgs::Pose>("target")};
// }

void BTWaitForTarget::halt() 
{
    std::cout << "ExecutePlan::halt" << std::endl;
    _halted = true;
    CoroActionNode::halt();
}
*/

bool comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion, double delta_orientation)
{

  if (  abs(pose1.position.x-pose2.position.x ) <= delta_posistion
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

BT::NodeStatus BTWaitForTarget::tick()
{
  // if( !getInput<geometry_msgs::Pose>("targetin", _target) )
  // {
  //   // no target go back to wait for target
  //   std::cout << "[ BTWaitForTarget: No goal ]" << std::endl;
  //   // return BT::NodeStatus::FAILURE;  
  //   return BT::NodeStatus::FAILURE;
  // }
  _success = false;
  setOutput<bool>("state", _success);
  while (!_aborted)
  {
    _pretarget = _target;
    if(getInput<geometry_msgs::Pose>("targetin", _target) )
    { 
      if (!comparePoses (_pretarget, _target, 0.00, 0.00))
      {
        break;
      }
    }
    // SleepMS(100);
    // std::cout << "BTWaitForTarget: Waiting" << std::endl;    
  }
  std::cout << "BTWaitForTarget: SUCCESS"<< std::endl;
  setOutput<geometry_msgs::Pose>("targetout", _target);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTPathPlanning::tick()
{
  RobotFunction robot_obj(_nh);
  // auto res = getInput<geometry_msgs::Pose>("target");
  if( !getInput<geometry_msgs::Pose>("goal", _target) )
  {
    // no target go back to wait for target
    std::cout << "[ BTPathPlanning: No goal ]" << std::endl;
    // return BT::NodeStatus::FAILURE;  
    throw BT::RuntimeError("missing required input [foal]");
  }

    // Reset this flag
  _aborted = false;
  printf("[ BTPathPlanning: STARTED ]. Target: x=%.2f y=%.2f z=%.2f w=%.2f\n", _target.position.x, _target.position.y, _target.position.z, _target.orientation.w);
  _counter = 0;
  pathplan planpath = robot_obj.PathPlanning(_target,  _move_group);
  _success = planpath.success;
  // while (!_aborted && _counter++ < 25)
  //   while (!_aborted)
  // {
  //   if (_success)
  //   {
  //     break;
  //   }
  //   // SleepMS(100);
  //   std::cout << "BTPathPlanning: Waiting" << std::endl;    
  // }

  if (_aborted) 
  {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    return BT::NodeStatus::FAILURE;
  }

  if (!_success) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput<moveit::planning_interface::MoveGroupInterface::Plan>("makeplan", planpath.plan);
  return BT::NodeStatus::SUCCESS; 
}

void BTPathPlanning::halt()
{
  _aborted = true;
}


BT::NodeStatus BTFollowPath::tick()
{
  RobotFunction robot_obj(_nh);
  // auto res = getInput<geometry_msgs::Pose>("target");
  if( !getInput<moveit::planning_interface::MoveGroupInterface::Plan>("planedplan", _myplan) )
  {
    // no target go back to wait for target
    std::cout << "[ BTFollowPath: No plan ]" << std::endl;
    // return BT::NodeStatus::FAILURE;  
    throw BT::RuntimeError("missing required input [plan]");
  }

    // Reset this flag
  _aborted = false;
  _success = false;
  _counter = 0;
  setOutput<bool>("state", _success);
  
  _success = robot_obj.MoveGroupExecutePlan(_move_group, _myplan);
  
  if (_aborted) 
  {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    return BT::NodeStatus::FAILURE;
  }

  if (!_success)
  {
    std::cout << "[BTFollowPath: %b]" << _success << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  std::cout << "[BTFollowPath: %b]" << _success << std::endl;
    setOutput<bool>("state", _success);

  return BT::NodeStatus::SUCCESS; 
}

void BTFollowPath::halt()
{
  _aborted = true;
}



BT::NodeStatus BTCameraFindTarget::tick()
{
  RobotFunction robot_obj(_nh);
  gettarget subtarget;
  subtarget.success = false;
  subtarget = robot_obj.CameraFindTarget();
  while (!_aborted)
  {
    subtarget = robot_obj.CameraFindTarget();
    if(subtarget.success)
    {
      break;
    }
  }
  std::cout << "BTCameraFindTarget: SUCCESS"<< std::endl;
  setOutput<geometry_msgs::Pose>("targetout", subtarget.target_pose);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTCloseToTarget::tick()
{
  RobotFunction robot_obj(_nh);
  gettarget subtarget;
  subtarget.success = false;
  _counter = 0;
  _execute_state = false;
  if( !getInput<geometry_msgs::Pose>("targetin", _target) || !getInput<double>("height", _height))
  {
    // no target go back to wait for target
    std::cout << "[ BTCloseToTarget: No target]" << std::endl;
    // return BT::NodeStatus::FAILURE;  
    throw BT::RuntimeError("missing required input [targetin]");
  }

   if (!getInput<bool>("state", _execute_state) )
   {
     _execute_state = false;
   }

  subtarget = robot_obj.KeepDistanceToTarget(_target, _height);
  // setOutput<geometry_msgs::Pose>("targetout", subtarget.target_pose);
  std::cout << "[ BTCloseToTarget: waiting for execute :]" << _execute_state << std::endl;
  while (!_aborted &&  _counter++ < 50)
  {
    /*
    if (robot_obj.comparePoses(_move_group, _target, 0.01, 0.01))
    {
      break;
    }
    else
    {
      SleepMS(100);
    }
    */
   if ( getInput<bool>("state", _execute_state) )
   {
     if (_execute_state == true)
     {
       std::cout << "[ BTCloseToTarget: Finish execute:]" << _execute_state << std::endl;
      //  setOutput<bool>("state", _execute_state);
      std::cout << "BTCloseToTarget: SUCCESS"<< std::endl;
      setOutput<geometry_msgs::Pose>("targetout", subtarget.target_pose);
        return BT::NodeStatus::SUCCESS;
      break;
     }
     else
     {
       SleepMS(100);
     }
     
   }
   else
   {
     SleepMS(100);
   }

  }

        return BT::NodeStatus::FAILURE;

}