#ifndef BTNODES_CONDITION_H_
#define BTNODES_CONDITION_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "robot_function/robot_function.h"
#include "robot_function/btnodes_arm.h"


namespace BTNodesCondition
{

// ACheckConditionArm
// input: target 
class ACheckConditionArm : public BT::CoroActionNode
{
  public:
    ACheckConditionArm(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, 
    moveit::planning_interface::MoveGroupInterface *move_group) :
        BT::CoroActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
    }

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
    {
      return { 
        BT::InputPort<geometry_msgs::Pose>("goal"),
        BT::InputPort<ParamType>("param"),
        BT::InputPort<int>("step")
        };
    }

  private:
    bool _aborted;
    geometry_msgs::Pose _goal;
    ParamType _param;
    gettarget _subtarget;
    int _step;
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_move_group;

};


// ACheckConditionFlag
// input:flag true: need help return failure 
class ACheckConditionFlag : public BT::SyncActionNode
{
  public:
    ACheckConditionFlag(const std::string& name, const BT::NodeConfiguration& config) :
        BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts()
    {
      return { 
        BT::InputPort<ParamType>("param"),
        BT::InputPort<int>("task") 
        };
    }

  private:
    ParamType _param;
    int _task;
};

class ACheckConditionLoad: public BT::SyncActionNode
{
  public:
    ACheckConditionLoad(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
    }
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return
      { 
        BT::InputPort<bool>("oneparam")
      };
    } 

  private:
    bool _load;
};


inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

}

}

#endif