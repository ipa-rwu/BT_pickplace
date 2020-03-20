#include "robot_function/btnodes_param.h"

BT_REGISTER_NODES(factory)
{
    BTNodesParameter::RegisterNodes(factory);
}


namespace BTNodesParameter
{


BT::NodeStatus AReloadParam::tick()
{
  // #!getInput<bool>("initial", _first) ||
  if(  !getInput<std::string>("type", _type))
  {
    throw BT::RuntimeError("missing required input [type]");
  }

  while(!_aborted)
  {
    if (!_first)
    {
      system("/home/rachel/kogrob/kogrob_ws/src/dynamic_tutorials/src/nodes/test.sh");
    } 
    else
    {
      setOutput<bool>("initial", false);
    }

    if (_type == "arm")
    {
      if (_paramcli_obj.get_param_arm(_nh, _param_temp_arm, _size))
      {
        _param.name = "arm";
        for (int i = 0; i < _size; i++)
        {
          _param.arm.param[i] = _param_temp_arm[i];
        }
        setOutput<ParamType>("param", _param);
      }
    }

    if (_type == "gripper")
    {
      if ( _paramcli_obj.get_param_gripper(_nh, _param_temp_gripper, _size) )
      {
        _param.name = "gripper";
        for (int i = 0; i < _size; i++)
        {
          _param.gripper.param[i] = _param_temp_gripper[i];
        }
        setOutput<ParamType>("param", _param);
      }
    }

    if (_type == "flag")
    {
      if (_paramcli_obj.get_param_flag(_nh, _param_temp_flag, _size_1))
      {
        _param.name = "flag";
        for (int i = 0; i < _size_1; i++)
        {
          _param.flag.param[i] = _param_temp_flag[i];
        }
        setOutput<ParamType>("param", _param);
      }
    }
    // robot_obj.PubFakeHoldObj();

    return BT::NodeStatus::SUCCESS;
  }
 } 

BT::NodeStatus ADistributeFlag::tick()
{
  if( !getInput<ParamType>("param", _param))
  {
    throw BT::RuntimeError("ASetFlag missing required input [param]");
  }

  // find a better way to set flag
  setOutput<bool>("flag0", _param.flag.param[0]);
  setOutput<bool>("flag1", _param.flag.param[1]);
  setOutput<bool>("flag2", _param.flag.param[2]);
  setOutput<bool>("flag3", _param.flag.param[3]);

  return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus ASetFlag::tick()
{
  if( !getInput<ParamType>("param", _param) || !getInput<int>("task", _task)
  || !getInput<bool>("value", _value))
  {
    throw BT::RuntimeError("ASetFlag missing required input [param] or [task]"); 
  }

  _param.flag.param[_task] = _value;
  // find a better way to set flag
  setOutput<ParamType>("flagupdate", _param);

  return BT::NodeStatus::SUCCESS;

}


}
