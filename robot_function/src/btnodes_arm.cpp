#include "robot_function/btnodes_arm.h"

BT_REGISTER_NODES(factory)
{
    BTNodesArm::RegisterNodes(factory);
}


namespace BTNodesArm
{

BT::NodeStatus APathPlanning::tick()
{
  RobotFunction robot_obj(_nh);
  _aborted = false;
  
  if( !getInput<TargetType>("goal", _goal) )
  {

    throw BT::RuntimeError("APathPlanning missing required input [goal]");
  }
  
  while(!_aborted)
  {
    if( getInput<TargetType>("goal", _goal) )
    {
        if(_goal.tag_name)
        {    
            _planpath = robot_obj.PathPlanning(_goal.Pose, _goal.Name, _move_group);
            std::cout << "APathPlanning: using name" << _goal.Name << std::endl;
            break;
        }
            
        if(_goal.tag_pose)
        {    
            _planpath = robot_obj.PathPlanning(_goal.Pose, _goal.Name, _move_group);
            printf("APathPlanning: Target pose: x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
            _goal.Pose.position.x, _goal.Pose.position.y, _goal.Pose.position.z, _goal.Pose.orientation.x, _goal.Pose.orientation.y, _goal.Pose.orientation.z, _goal.Pose.orientation.w);
            break;    
        }

        if(_goal.tag_waypoint)
        {
            _planpath = robot_obj.CartesianPathPlan( _goal.Waypoint, _move_group, _eef_step, _jump_threshold);
            printf("APathPlanning: Waypoint: x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
            _goal.Waypoint.position.x, _goal.Waypoint.position.y, _goal.Waypoint.position.z, _goal.Waypoint.orientation.x, _goal.Waypoint.orientation.y, _goal.Waypoint.orientation.z, _goal.Waypoint.orientation.w);
            break;  
        }
    }
  }
  // Reset this flag
  
    _success = _planpath.success;
    if (_aborted) 
    {
    // this happens only if method halt() was invoked
    return BT::NodeStatus::FAILURE;
    }

    if (!_success) 
    {
    return BT::NodeStatus::FAILURE;
    }
    setOutput<moveit::planning_interface::MoveGroupInterface::Plan>("plan", _planpath.plan);
    return BT::NodeStatus::SUCCESS; 
}


BT::NodeStatus AFollowPath::tick()
{
     RobotFunction robot_obj(_nh);
    _aborted = false;

    if( !getInput<moveit::planning_interface::MoveGroupInterface::Plan>("plan", _myplan) )
    {
        throw BT::RuntimeError("AFollowPath missing required input [plan]");
    }

    // Reset this flag

    _success = robot_obj.MoveGroupExecutePlan(_move_group, _myplan);
  
    if (_aborted) 
    {
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

// only accept pose or waypoint
// update pose 
// APreparePoseArm
// input: goal, param, targettype, step
// output: targetout
BT::NodeStatus APreparePoseArm::tick()
{
  RobotFunction robot_obj(_nh);
  _subtarget.success = false;
  
  while (!_aborted)
  {
        if( !getInput<geometry_msgs::Pose>("goalarm", _goal) || !getInput<ParamType>("param", _param) || 
            !getInput<std::string>("targettype", _targettype) || !getInput<int>("step", _step))
        {
         throw BT::RuntimeError("APreparePoseArm missing required input");
        }
        _step = _step - 1;
        _subtarget = robot_obj.KeepDistanceToTarget(_goal, _param.arm.param[_step]);
        
        printf("APreparePoseArm: step = %d\n",_step);
        
        std::cout << "APreparePoseArm height: "<< _param.arm.param[_step] << std::endl;

        std::cout << "APreparePoseArm gripper: "<< _param.gripper.param[_step] << std::endl;
       
        if(_subtarget.success)
        {
        
            if (_targettype == "pose")
            {
                _targetout.Pose = _subtarget.target_pose;
                _targetout.tag_pose = true;
                
                printf("APreparePoseArm: pose: x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
                _targetout.Pose.position.x, _targetout.Pose.position.y, _targetout.Pose.position.z,
                _targetout.Pose.orientation.x,_targetout.Pose.orientation.y,
                _targetout.Pose.orientation.z,_targetout.Pose.orientation.w);

                break;
            }

            if (_targettype == "waypoint")
            {
                _targetout.Waypoint = _subtarget.target_pose;
                _targetout.tag_waypoint = true;
                
                printf("APreparePoseArm: waypoint: x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
                _targetout.Waypoint.position.x, _targetout.Waypoint.position.y, 
                _targetout.Waypoint.position.z, _targetout.Waypoint.orientation.x,
                _targetout.Waypoint.orientation.y,_targetout.Waypoint.orientation.z,
                _targetout.Waypoint.orientation.w);

                // std::cout << "BTCloseToTarget waypoint height: "<< _height << std::endl;
                break;
            }   
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }         
    }
 
    /*    
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
        */     
  /*  
  std::cout << "BTCloseToTarget obs target name: "<< _obstarget.tag_name << std::endl;
  std::cout << "BTCloseToTarget obs target pose: "<< _obstarget.tag_pose << std::endl;
  std::cout << "BTCloseToTarget obs target waypoint: "<< _obstarget.tag_waypoint << std::endl;
  */
 
    std::cout << "APreparePoseArm target name: "<< _targetout.tag_name << std::endl;
    std::cout << "APreparePoseArm target pose: "<< _targetout.tag_pose << std::endl;
    std::cout << "APreparePoseArm target waypoint: "<< _targetout.tag_waypoint << std::endl;

    std::cout << "APreparePoseArm target z      : ]" << _targetout.Pose.position.z << std::endl;   
    // std::cout << "[ BT target height :  "<< _height << "]"<< std::endl;
    setOutput<TargetType>("targetout", _targetout);
    return BT::NodeStatus::SUCCESS;
}

}