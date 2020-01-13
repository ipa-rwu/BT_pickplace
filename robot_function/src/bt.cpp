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

BT::NodeStatus BTWaitForTarget::tick()
{
  RobotFunction robot_obj(_nh);
  gettarget subtarget;
  subtarget.success = _gettarget;
  subtarget = robot_obj.CameraFindTarget();
  while (!_aborted)
  {
    subtarget = robot_obj.CameraFindTarget();
    if(subtarget.success)
    {
      break;
    }
  }
  std::cout << "FindTarget: SUCCESS"<< std::endl;
  setOutput<geometry_msgs::Pose>("target", subtarget.target_pose);

  return BT::NodeStatus::SUCCESS;
}

/*
BT::NodeStatus BTPathPlanning::tick()
{
  // RobotFunction robot_obj(_nh);
  
  // if( !getInput<geometry_msgs::Pose>("goal", _target) )
  // {
  //   // no target go back to wait for target
  //   std::cout << "[ BTPathPlanning: No goal ]" << std::endl;
  //   // return BT::NodeStatus::FAILURE;  
  //   throw BT::RuntimeError("missing required input [foal]");
  // }

  // pathplan planpath = robot_obj.PathPlanning(_target);
  _success = _robot_obj.PathPlanning(_target, _move_group);
    if(_success)
    {
      std::cout << "PathPlanning: " << _success << std::endl;
      // setOutput("pathplan", planpath.plan);
      return BT::NodeStatus::SUCCESS;  
    }
    else
    {
      return BT::NodeStatus::FAILURE;  
    }
    
}

*/



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
    while (!_aborted)
  {
    if (_success)
    {
      break;
    }
    // SleepMS(100);
    std::cout << "PathPlanning: Waiting" << std::endl;    
  }

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
  _success = robot_obj.MoveGroupExecutePlan(_move_group, _myplan);
   
  // while (!_aborted && _counter++ < 25)
    while (!_aborted)
  {
    if (_success)
    {
      break;
    }
    // SleepMS(100);
    std::cout << "PathPlanning: Waiting" << std::endl;    
  }

  if (_aborted) 
  {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    return BT::NodeStatus::FAILURE;
  }

  if (!_success) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS; 
}

void BTFollowPath::halt()
{
  _aborted = true;
}