#ifndef BTNODES_PARAM_H_
#define BTNODES_PARAM_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "robot_function/parameter_client.h"



template <int m>
struct ParamArmType
{
  static const int length = m;
  double param[m];
};

template <int n>
struct ParamGripperType
{
  // std::string T2S1grip;
  // std::string T2S2grip;
  // std::string T2S3grip;
  // std::string T3S1grip;
  // std::string T3S2grip;
  // std::string T3S3grip;
  static const int length = n;
  std::string param[n];
};

template <int p>
struct FlagType
{
  static const int length = p;
  bool param[p];
};

struct ParamType
{
  //arm or gripper
  std::string name;
  ParamArmType<6> arm;
  ParamGripperType<6> gripper;
  FlagType<4> flag; 
};

namespace BT
{

template <> inline ParamType convertFromString(StringView str)
{
    auto parts = splitString(str, ';');
    ParamType param;
    if (convertFromString<std::string>(parts[0]) == "arm")
    {
      param.name = convertFromString<std::string>(parts[0]);
      for (int i = 0; i < param.arm.length; i++)
      {
        param.arm.param[i] = convertFromString<double>(parts[i+1]);
      }
      return param;
    }
    if (convertFromString<std::string>(parts[0]) == "gripper")
    {
      param.name = convertFromString<std::string>(parts[0]);
      for (int i = 0; i < param.gripper.length; i++)
      {
        param.gripper.param[i] = convertFromString<std::string>(parts[i+1]);
      }
      return param;
    }

    if (convertFromString<std::string>(parts[0]) == "flag")
    {
      param.name = convertFromString<std::string>(parts[0]);
      for (int i = 0; i < param.gripper.length; i++)
      {
        param.flag.param[i] = convertFromString<bool>(parts[i+1]);
      }
      return param;
    }
}
}


namespace BTNodesParameter
{
// TODO genertate from configuration

// AReloadParam:
// input: initial type param load
// output: param: .arm.param . gripper.param
class AReloadParam: public BT::CoroActionNode
{
  public:
    AReloadParam(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh,
    bool first, bool load)
    : BT::CoroActionNode(name,config), _nh(nh), _first(first), _load(load)
    {
      _aborted = false;
    }

    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    // TODO: generate from configuration
    static BT::PortsList providedPorts() 
    { 
      return{ 
        BT::InputPort<ParamType>("paramin"),
        BT::OutputPort<ParamType>("paramout"),
        BT::InputPort<bool>("bool"),
        BT::InputPort<std::string>("type"),
        BT::InputPort<bool>("initialin"),
        BT::OutputPort<bool>("initialout"),
        BT::InputPort<bool>("load"),
        BT::OutputPort<bool>("load") };
    } 

  private:
    bool _aborted;
    bool _first;
    bool _load;
    std::string _type;
    ParamClient _paramcli_obj;
    ParamType _param;
    ros::NodeHandle _nh;
    static const int _size = 6;
    static const int _size_1 = 4;
    double _param_temp_arm[_size] = {0.00};
    std::string _param_temp_gripper[_size] = {"none"};
    bool _param_temp_flag[_size_1] = {false};
};


//0:FHelp 1:FFindObj 2:FPicked 3:FPlaced 
class ADistributeFlag: public BT::SyncActionNode
{
  public:
    ADistributeFlag(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
    }
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return
      { 
        BT::InputPort<ParamType>("param"),
        BT::OutputPort<bool>("flag0"),
        BT::OutputPort<bool>("flag1"),
        BT::OutputPort<bool>("flag2"),
        BT::OutputPort<bool>("flag3")

      };
    } 

  private:

    ParamType _param;
};

// ASetFlag
// input: param task value
// output flagupdate
class ASetFlag: public BT::SyncActionNode
{
  public:
    ASetFlag(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
    }
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return
      { 
        BT::InputPort<ParamType>("param"),
        BT::InputPort<bool>("value"),
        BT::InputPort<int>("task"),
        BT::OutputPort<ParamType>("flagupdate")
        // ,
        // BT::OutputPort<bool>("flag0"),
        // BT::OutputPort<bool>("flag1"),
        // BT::OutputPort<bool>("flag2"),
        // BT::OutputPort<bool>("flag3")

      };
    } 

  private:
    int _task;
    bool _value;
    ParamType _param;
};


class ASetMarker: public BT::SyncActionNode
{
  public:
    ASetMarker(const std::string& name, const BT::NodeConfiguration& config, int marker)
    : BT::SyncActionNode(name, config),  _marker(marker)
    {
    }
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return
      { 
        BT::InputPort<int>("markerin"),
        BT::OutputPort<int>("markerout")

      };
    } 

  private:
    int _marker;

};

class AReadParam: public BT::SyncActionNode
{
  public:
    AReadParam(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
    }
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return
      { 
        BT::InputPort<ParamType>("paramin")
      };
    } 

  private:
    ParamType _param;

};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

    // factory.registerNodeType<BTPathPlanning>("BTPathPlanning");

}

}

#endif /* BTNODES_PARAM_H_ */
