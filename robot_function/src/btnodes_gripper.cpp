#include "robot_function/btnodes_gripper.h"

BT_REGISTER_NODES(factory)
{
    BTNodesGripper::RegisterNodes(factory);
}


namespace BTNodesGripper
{

BT::NodeStatus APrepareGripper::tick()
{
    // if( !getInput<std::string>("command", _commandin) )
    if( !getInput<ParamType>("param", _param) || !getInput<int>("step", _step))
    {
        throw BT::RuntimeError("AGripperMove missing required input [gripper command]");
    }

    _step = _step - 1;

    _command = _param.gripper.param[_step];
     setOutput<std::string>("command", _command);
    return BT::NodeStatus::SUCCESS;
  
}


// AGripperMove
// input: command step
BT::NodeStatus AGripperMove::tick()
{
    SleepMS(500);
    GripperFunction _gripper_obj;
    // if( !getInput<std::string>("command", _commandin) )
    if( !getInput<std::string>("command", _command))
    {
        throw BT::RuntimeError("AGripperMove missing required input [gripper command]");
    }

    if(_command == "open")
    {
        _gripper_obj.MoveGripper(_gripper_group, _command);              
        return BT::NodeStatus::SUCCESS;

    }

    if(_command == "close")
    {
        _gripper_obj.MoveGripper(_gripper_group, _command);
        return BT::NodeStatus::SUCCESS;
    }

    if(_command == "none")
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;

  
}


BT::NodeStatus AGripperMoveSchunk::tick()
{
    SleepMS(500);
    GripperFunction _gripper_obj;
    if( !getInput<std::string>("command", _commandin) )
    {
        throw BT::RuntimeError("AGripperMoveSchunk missing required input [gripper command]");
    }

    if(_commandin == "open")
    {
        _result = _gripper_obj.GripperOpen(_nh);
        if (_result)
        {
        return BT::NodeStatus::SUCCESS;
        }
        else
        {
        return BT::NodeStatus::FAILURE;
        }
    }

    if(_commandin == "close")
    {
        _result = _gripper_obj.GripperClose(_nh);
        if (_result)
        {
        return BT::NodeStatus::SUCCESS;
        }
        else
        {
        return BT::NodeStatus::FAILURE;
        }
    }

        if(_commandin == "none")
    {
        return BT::NodeStatus::SUCCESS;
    }
}


}