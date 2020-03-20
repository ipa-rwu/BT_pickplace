#include "robot_function/btnodes_condition.h"

BT_REGISTER_NODES(factory)
{
    BTNodesCondition::RegisterNodes(factory);
}


namespace BTNodesCondition 
{
    
BT::NodeStatus ACheckConditionArm::tick()
{
    RobotFunction robot_obj(_nh);
    if( !getInput<geometry_msgs::Pose>("goal", _goal) || !getInput<ParamType>("param", _param) || 
        !getInput<int>("step", _step))
    {
      throw BT::RuntimeError("ACheckConditionArm missing required input [targetin]");
    }
    // if (!getInput<double>("heightin", _height))
    // {
    //   throw BT::RuntimeError("BTCheckCondition missing required input [height]");

    // }

    // if(_targetin.tag_pose)
    // {
    //   _subtarget = robot_obj.KeepDistanceToTarget(_targetin.Pose, _height);
    // }

    // if(_targetin.tag_waypoint)
    // {
    //   _subtarget = robot_obj.KeepDistanceToTarget(_targetin.Waypoint, _height);
    // }

     _subtarget = robot_obj.KeepDistanceToTarget(_goal, _param.arm.param[_step]);

     if (_subtarget.success)
     {
        if (robot_obj.comparePoses(_move_group, _subtarget.target_pose, 0.0250, 0.0250))
        {
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE;

     }

    return BT::NodeStatus::FAILURE;
}


BT::NodeStatus ACheckConditionFlag::tick()
{
    if( !getInput<ParamType>("param", _param) || !getInput<int>("task", _task))
    {
        // throw BT::RuntimeError("ACheckConditionFlag missing required input [param] or [task]"); 
        return BT::NodeStatus::FAILURE;

    }

    if (_param.flag.param[_task])
    {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}


BT::NodeStatus ACheckConditionLoad::tick()
{
    if( !getInput<bool>("oneparam", _load ) )
    {
        // throw BT::RuntimeError("ACheckConditionFlag missing required input [param] or [task]"); 
        return BT::NodeStatus::FAILURE;

    }

    if (_load)
    {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}

}