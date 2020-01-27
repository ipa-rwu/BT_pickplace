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
    if( !getInput<TargetType>("targetin", _targetin))
    {
      throw BT::RuntimeError("BTCheckCondition missing required input [targetin]");
    }
    if (!getInput<double>("heightin", _height))
    {
      throw BT::RuntimeError("BTCheckCondition missing required input [height]");

    }

    if(_targetin.tag_pose)
    {
      _subtarget = robot_obj.KeepDistanceToTarget(_targetin.Pose, _height);
    }

    if(_targetin.tag_waypoint)
    {
      _subtarget = robot_obj.KeepDistanceToTarget(_targetin.Waypoint, _height);
    }

    if (robot_obj.comparePoses(_move_group, _subtarget.target_pose, 0.005, 0.005))
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
  while (!_aborted) 
  {
  
      if( getInput<TargetType>("targetin", _targetin))
      {
        // throw BT::RuntimeError(" BTWaitForTarget missing required input [targetin]");
      
      _targetout = _targetin;
      if (_targetout.tag_name == true)
      {
        // std::cout << "BTWaitForTarget got target name: "<< _targetout.Name << std::endl;
        break;
      }
      if (_targetout.tag_pose == true)
      {
        // std::cout << "BTWaitForTarget got target pose: "<< _targetout.Pose.position.z << std::endl;
        break;
      }
      if (_targetout.tag_waypoint == true)
      {
        // std::cout << "BTWaitForTarget got target waypoint: "<< _targetout.Waypoint.position.z  << std::endl;
        break;
      }    
    }
  }

  std::cout << "BTWaitForTarget  target name: "<< _targetout.tag_name << std::endl;
  std::cout << "BTWaitForTarget target pose: "<< _targetout.tag_pose << std::endl;
  std::cout << "BTWaitForTarget target waypoint: "<< _targetout.tag_waypoint << std::endl;
  setOutput<TargetType>("targetnameout", _targetout);
  return BT::NodeStatus::SUCCESS;

}



BT::NodeStatus BTPathPlanning::tick()
{
  RobotFunction robot_obj(_nh);
  _aborted = false;
  // auto res = getInput<geometry_msgs::Pose>("target");
  // must define target name 
  
  if( !getInput<TargetType>("goal", _goal) )
  {
    // no target go back to wait for target
    // return BT::NodeStatus::FAILURE;  
    throw BT::RuntimeError("BTPathPlanning missing required input [goal]");
  }
  
  while(!_aborted)
  {
    if( getInput<TargetType>("goal", _goal) )
    {
    if(_goal.tag_name)
    {    
      _planpath = robot_obj.PathPlanning(_goal.Pose, _goal.Name, _move_group);
      std::cout << "BTPathPlanning: using name" << _goal.Name << std::endl;
      break;
    }
        
    if(_goal.tag_pose)
    {    
      _planpath = robot_obj.PathPlanning(_goal.Pose, _goal.Name, _move_group);
      printf("BTPathPlanning: Target pose: x=%.2f y=%.2f z=%.2f w=%.2f\n",
      _goal.Pose.position.x, _goal.Pose.position.y, _goal.Pose.position.z, _goal.Pose.orientation.w);
      break;    
    }
    
    if(_goal.tag_waypoint)
    {
      _planpath = robot_obj.CartesianPathPlan( _goal.Waypoint, _move_group, _eef_step, _jump_threshold);
      printf("BTPathPlanning: Waypoint: x=%.2f y=%.2f z=%.2f w=%.2f\n",
      _goal.Waypoint.position.x, _goal.Waypoint.position.y, _goal.Waypoint.position.z, _goal.Waypoint.orientation.w);
      break;  
    }
    }
  }

  // Reset this flag
  
  _success = _planpath.success;
  if (_aborted) 
  {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    return BT::NodeStatus::FAILURE;
  }

  if (!_success) 
  {
    return BT::NodeStatus::FAILURE;
  }
  setOutput<moveit::planning_interface::MoveGroupInterface::Plan>("makeplan", _planpath.plan);
  return BT::NodeStatus::SUCCESS; 
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
  _counter = 0;


  _success = robot_obj.MoveGroupExecutePlan(_move_group, _myplan);
  
  if (_aborted) 
  {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    return BT::NodeStatus::FAILURE; 
  }

  if (!_success)
  {
    std::cout << "[BTFollowPath: FAILED]" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  std::cout << "[BTFollowPath: SUCCESS]" << _success << std::endl;

  return BT::NodeStatus::SUCCESS; 
}


BT::NodeStatus BTCheckGripperCommand::tick()
{
  if( !getInput<std::string>("commandin", _commandin) )
  {
    throw BT::RuntimeError("missing required input [commandin]");
  }
  std::cout << "[ BTCheckGripperCommand: ]" << _commandin << std::endl;  
    if( _commandin == "open" )
    {
      setOutput<std::string>("commandout", _commandin);
      std::cout << "[ BTCheckGripperCommand: open ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }
    if( _commandin == "close" )
    {
      setOutput<std::string>("commandout", _commandin);
      std::cout << "[ BTCheckGripperCommand: close ]" << std::endl;
        return BT::NodeStatus::SUCCESS;

    }
    if( _commandin == "no" )
    {
      std::cout << "[ BTCheckGripperCommand: No command for gripper ]" << std::endl;  
      return BT::NodeStatus::SUCCESS;
    }        
}


BT::NodeStatus BTGripperMove::tick()
{
   GripperFunction _gripper_obj;
  if( !getInput<std::string>("commandin", _commandin) )
  {
    throw BT::RuntimeError("missing required input [gripper command]");
  }

  std::cout << "[ BTGripperMove: " << _commandin << std::endl;  

    if (_gripper_obj.MoveGripper(_gripper_group, _commandin))
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
  
}


BT::NodeStatus BTStringtoPose::tick()
{
  
    if ( !getInput<PositionGo>("stringin", _goal))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }

	std::cout << "[ container: " << _goal.pz << std::endl;  
	_target_pose.position.x = _goal.px;
	_target_pose.position.y = _goal.py;
	_target_pose.position.z = _goal.pz;
  _target_pose.orientation.x = _goal.ox;
  _target_pose.orientation.y = _goal.oy;
  _target_pose.orientation.z = _goal.oz;
  _target_pose.orientation.w = _goal.ow;
  std::cout << "[ container: " << _target_pose.position.z << std::endl;
  setOutput<geometry_msgs::Pose>("targetpose", _target_pose);
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus BTStringtoTarget::tick()
{
  
    if ( !getInput<std::string>("stringin", _stringin))
    {
        throw BT::RuntimeError("missing required input [string]");
    }
  _targetout.Name = _stringin;
  _targetout.tag_name = true;
  setOutput<TargetType>("targetpose", _targetout);
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus BTCameraFindTarget::tick()
{
  RobotFunction robot_obj(_nh);
  _subtarget.success = false;
  // subtarget.success = true;
	/*
  PositionGo goal;
    if ( !getInput<PositionGo>("targetin", goal))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }

	geometry_msgs::Pose target_pose;
	target_pose.position.x = goal.px;
	target_pose.position.y = goal.py;
	target_pose.position.z = goal.pz;
  target_pose.orientation.w=1.0;
  */

  _subtarget = robot_obj.CameraFindTarget();
  while (!_aborted)
  {
    _subtarget = robot_obj.CameraFindTarget();
    setOutput<bool>("tagisobjpose", false);
    if(_subtarget.success)
    {
      _target.tag_pose = _subtarget.success;
      break;
    }
  }
  std::cout << "BTCameraFindTarget: SUCCESS"<< std::endl;
  _target.Pose = _subtarget.target_pose;
  
  std::cout << "BTCameraFindTarget  target name: "<< _target.tag_name << std::endl;
  std::cout << "BTCameraFindTarget target pose: "<< _target.tag_pose << std::endl;
  std::cout << "BTCameraFindTarget target waypoint: "<< _target.tag_waypoint << std::endl;
  
  setOutput<TargetType>("targetout", _target);
  setOutput<bool>("tagisobjposeout", true);
  // setOutput<geometry_msgs::Pose>("targetout", target_pose);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTIsObjPose::tick()
{
  if( !getInput<bool>("tagisobjposein", _tagisobjpose) )
  {
    _tagisobjpose = false;
    return BT::NodeStatus::FAILURE; 
  }
  if( _tagisobjpose )
  {
    if( !getInput<TargetType>("targetin", _objpose))
    {
    throw BT::RuntimeError("missing required input [plan]");
   }
    setOutput<TargetType>("targetout", _objpose);
    return BT::NodeStatus::SUCCESS;
  }
  if ( !_tagisobjpose )
  {
    return BT::NodeStatus::FAILURE; 
  }
}


BT::NodeStatus BTIsHoldObj::tick()
{
  /*
  RobotFunction robot_obj(_nh);
  bool isholdobj;
  isholdobj = false;
  isholdobj = robot_obj.IsHoldObj();
  while (!_aborted)
  {
    isholdobj = robot_obj.IsHoldObj();
    // std::cout << "BTIsHoldObj: "<< isholdobj<<std::endl;

    if(!isholdobj)
    {
      break;
    }
  }
  std::cout << "BTIsHoldObj: SUCCESS"<< std::endl;
  return BT::NodeStatus::SUCCESS;
  */
  if( !getInput<bool>("tagisholdin", _tagishold) || !_tagishold)
  {
    _tagishold = false;
    return BT::NodeStatus::FAILURE; 
  }
  if( _tagishold )
  {
    return BT::NodeStatus::SUCCESS;
  }      
}

BT::NodeStatus BTIsObjContainer::tick()
{
  if( !getInput<bool>("tagisobjconin", _tagisobjcon))
  {
    _tagisobjcon = false;
    std::cout << "BTIsObjContainer missing required input [tagisobjcon]" << std::endl;
  }
  if (!_tagisobjcon)
  {
    return BT::NodeStatus::FAILURE; 

  }
  if( _tagisobjcon )
  {
    return BT::NodeStatus::SUCCESS;
  }  
}

// only accept pose or waypoint
// update pose 
BT::NodeStatus BTCloseToTarget::tick()
{
  RobotFunction robot_obj(_nh);
  _subtarget.success = false;
  
  while (!_aborted)
  {
        if( !getInput<TargetType>("targetin", _obstarget) || !getInput<double>("height", _height) || !getInput<std::string>("targettype", _targettype))
        {
         throw BT::RuntimeError("BTCloseToTarget missing required input [targetin]");
        }
        if (_obstarget.tag_pose == true)
        {
          
          _subtarget = robot_obj.KeepDistanceToTarget(_obstarget.Pose, _height);   
          if(_subtarget.success)
          {
            
            if (_targettype == "pose")
            {
              _targetout.Pose = _subtarget.target_pose;
              _targetout.tag_pose = true;
              std::cout << "BTCloseToTarget pose height: "<< _height << std::endl;
              break;
            }

            if (_targettype == "waypoint")
            {
              _targetout.Waypoint = _subtarget.target_pose;
              _targetout.tag_waypoint = true;
              std::cout << "BTCloseToTarget waypoint height: "<< _height << std::endl;
              break;
            }            
          }
 
        }
        if (_obstarget.tag_waypoint == true)
        {
          std::cout << "BTCloseToTarget height: "<< _height << std::endl;
          _subtarget = robot_obj.KeepDistanceToTarget(_obstarget.Waypoint, _height);   
          if(_subtarget.success)
          {
            _targetout.Waypoint = _subtarget.target_pose;
            _targetout.tag_waypoint = true;          
            break;         
          }
        }     
  }
  /*  
  std::cout << "BTCloseToTarget obs target name: "<< _obstarget.tag_name << std::endl;
  std::cout << "BTCloseToTarget obs target pose: "<< _obstarget.tag_pose << std::endl;
  std::cout << "BTCloseToTarget obs target waypoint: "<< _obstarget.tag_waypoint << std::endl;
  */
 
  std::cout << "BTCloseToTarget target name: "<< _targetout.tag_name << std::endl;
  std::cout << "BTCloseToTarget target pose: "<< _targetout.tag_pose << std::endl;
  std::cout << "BTCloseToTarget target waypoint: "<< _targetout.tag_waypoint << std::endl;

  std::cout << "BTCloseToTarget target z      : ]" << _targetout.Pose.position.z << std::endl;   
  // std::cout << "[ BT target height :  "<< _height << "]"<< std::endl;
  setOutput<TargetType>("targetout", _targetout);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BTAdvertiseGripperCommand::tick()
{
  if( !getInput<std::string>("commandin", _command) )
  {
    throw BT::RuntimeError("missing required input [plan]");
  }
  setOutput<std::string>("commandout", _command);
  return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus BTPubFakeHoldObj::tick()
{
  RobotFunction robot_obj(_nh);
  while(!_aborted)
  {
    robot_obj.PubFakeHoldObj();
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus BTStringToBool::tick()
{
  if( !getInput<std::string>("stringin", _string) )
  {
    throw BT::RuntimeError("missing required input [plan]");
  }
  if( _string == "true")
  {
    _bool = true;
    setOutput<bool>("boolout", _bool);
    
  }
  if( _string == "false")
  {
    _bool = false;
    setOutput<bool>("boolout", _bool);
    
  }
  return BT::NodeStatus::SUCCESS;
}