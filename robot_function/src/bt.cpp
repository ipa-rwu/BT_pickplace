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

/*
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
*/

bool comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion)
{

  if ( 
      abs(pose1.position.z-pose2.position.z ) <= delta_posistion

     )
  {
    return true;
  }
  else
  {
    return false;
  }
}

BT::NodeStatus BTCheckCondition::tick()
{
    RobotFunction robot_obj(_nh);
    if( !getInput<geometry_msgs::Pose>("targetin", _obstarget) || !getInput<double>("heightin", _height))
    {
      throw BT::RuntimeError("missing required input [targetin]");
    }
    gettarget subtarget = robot_obj.KeepDistanceToTarget(_obstarget, _height);
    if (robot_obj.comparePoses(_move_group, subtarget.target_pose, 0.05, 0.05))
    {
      return BT::NodeStatus::SUCCESS;
    }
      return BT::NodeStatus::FAILURE;
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
  _stateexecutefree = true;
  _statewaitfortarget = true;
  setOutput<bool>("statewaittarget", _statewaitfortarget);
  std::cout << "[ BT_state wait for target: ]" << _statewaitfortarget << std::endl;
  setOutput<bool>("stateexecutefree", _stateexecutefree);
  while (!_aborted)
  {
    _pretarget = _target;
    if(getInput<geometry_msgs::Pose>("targetin", _target) )
    { 
          std::cout << "BTWaitForTarget: got" << std::endl;    

              break;

      // if (!comparePoses (_pretarget, _target, 0.00, 0.00))
      // {
      //   break;
      // }
      // else
      // {
      //   return BT::NodeStatus::FAILURE;
      // }
      
    }
  }
  std::cout << "BTWaitForTarget: SUCCESS"<< std::endl;
  _statewaitfortarget = false;
  setOutput<geometry_msgs::Pose>("targetout", _target);
  setOutput<bool>("statewaittarget", _statewaitfortarget);
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
  _statewaitfortarget = false;
  _stateexecutefree = false;
  _counter = 0;
   setOutput<bool>("statewaittarget", _statewaitfortarget);
   setOutput<bool>("stateexecutefree", _stateexecutefree);

  _success = robot_obj.MoveGroupExecutePlan(_move_group, _myplan);
  
  if (_aborted) 
  {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    _stateexecutefree = true;   
    setOutput<bool>("stateexecutefree", _stateexecutefree);
    return BT::NodeStatus::FAILURE; 
  }

  if (!_success)
  {
    std::cout << "[BTFollowPath: FAILED]" << std::endl;
    _stateexecutefree = true;
    setOutput<bool>("stateexecutefree", _stateexecutefree);
    return BT::NodeStatus::FAILURE;
  }
  std::cout << "[BTFollowPath: SUCCESS]" << _success << std::endl;
  _stateexecutefree = true;
  // _statewaitfortarget = true;
  setOutput<bool>("stateexecutefree", _stateexecutefree);
  // setOutput<bool>("statewaittarget", _statewaitfortarget);

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
  gettarget presubtarget;
  subtarget.success = false;
  _counter = 0;
  _pretarget = _obstarget;
  _preheight = _height;
  std::cout << "[ BT CLoseToTarget : I am here]"<< std::endl;
  while (!_aborted)
  {
        if( !getInput<geometry_msgs::Pose>("targetin", _obstarget) || !getInput<double>("height", _height))
        {
         throw BT::RuntimeError("missing required input [targetin]");
        }
       else
        {
          subtarget = robot_obj.KeepDistanceToTarget(_obstarget, _height);
          std::cout << "[ BT target z      : ]" << _obstarget.position.z << std::endl;
          std::cout << "[ BT target height :  "<< _height << "]"<< std::endl;
          setOutput<geometry_msgs::Pose>("targetout", subtarget.target_pose);
          return BT::NodeStatus::SUCCESS;
        }
       
      // else
      // {
      //   return BT::NodeStatus::FAILURE;  
      // }
          
    
       
  }
  
  return BT::NodeStatus::FAILURE;  
}