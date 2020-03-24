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
      throw BT::RuntimeError("AReloadParamArm missing required input [type]");
    }

    if ( !getInput<bool>("initialin", _first))
    {
      setOutput<bool>("initialout", false);
    }

    getInput<ParamType>("paramin", _param);

    std::cout << "AReloadParamArm first: "<< _first << std::endl;
    while(!_aborted)
    {
      if (!_first)
      {
        // system("/home/rachel/kogrob/kogrob_ws/src/dynamic_parameter/src/nodes/test.sh");
      } 


      if (_type == "arm")
      {
        if (_paramcli_obj.get_param_arm(_nh, _param_temp_arm, _size))
        {
          _param.name = "arm";
          for (int i = 0; i < _size; i++)
          {
            _param.arm.param[i] = _param_temp_arm[i];
            std::cout << "AReloadParamArm arm: "<< _param.arm.param[i] << std::endl;
          }
          setOutput<ParamType>("paramout", _param);
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
            std::cout << "AReloadParamArm gripper: "<< _param.gripper.param[i] << std::endl;
          }
          setOutput<ParamType>("paramout", _param);
        }
      }

      if (_type == "flag")
      {
        if (!_first)
        {
          system("/home/rachel/kogrob/kogrob_ws/src/dynamic_parameter/src/nodes/test.sh");
        }        
       if (_paramcli_obj.get_param_flag(_nh, _param_temp_flag, _size_1))
        {
          _param.name = "flag";
          for (int i = 0; i < _size_1; i++)
          {
            _param.flag.param[i] = _param_temp_flag[i];
            std::cout << "AReloadParamArm flag: "<< _param.flag.param[i] << std::endl;
          }
          setOutput<ParamType>("paramout", _param);
        }
      }
      // robot_obj.PubFakeHoldObj();
      if (!  getInput<bool>("load", _load))
      {
            setOutput<bool>("load", true);

      }
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

BT::NodeStatus ASetMarker::tick()
{
  if( !getInput<int>("markerin", _marker))
  {
    // throw BT::RuntimeError("ASetFlag missing required input [param] or [task]"); 
  }
  
  if (_marker == 1)
  {
    _marker = 2;
    // std::cout << "ASetMarker : "<< _marker << std::endl;
    setOutput<int>("markerout", _marker);
    return BT::NodeStatus::SUCCESS;

  }

 if (_marker == 2)
  {
    _marker = 1;
    setOutput<int>("markerout", _marker);
    return BT::NodeStatus::SUCCESS;

  }
  
  return BT::NodeStatus::SUCCESS;

}


BT::NodeStatus AReadParam::tick()
{
  if( !getInput<ParamType>("paramin", _param))
  {
    // throw BT::RuntimeError("ASetFlag missing required input [param] or [task]"); 
  }
  
        for (int i = 0; i < 6; i ++ )
        {
            std::cout << "AReadParam height: "<< _param.arm.param[i] << std::endl;

        }
        for (int i = 0; i < 6; i ++ )
        {
            std::cout << "AReadParam gripper: "<< _param.gripper.param[i] << std::endl;

        }
                for (int i = 0; i < 4; i ++ )
        {
            std::cout << "AReadParam flag: "<< _param.flag.param[i] << std::endl;

        }

}


}
