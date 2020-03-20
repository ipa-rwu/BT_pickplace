#ifndef BTNODES_GRIPPER_H_
#define BTNODES_GRIPPER_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "robot_function/robot_gripper.h"
#include "robot_function/btnodes_param.h"


inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


namespace BTNodesGripper
{
// AGripperMove
// input: step command
//  
class AGripperMove: public BT::CoroActionNode
{
  public:
    AGripperMove(const std::string& name, const BT::NodeConfiguration& config, 
    ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group)
    : BT::CoroActionNode(name,config),_nh(nh), _gripper_group(gripper_group)
    {
        _aborted = false;
    }
    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    static BT::PortsList providedPorts() 
    { 
      return{ 
        // BT::InputPort<std::string>("command")
        BT::InputPort<std::string>("command")
      };
    } 

    private:
      std::string _command;
      bool _aborted; 
      ros::NodeHandle _nh;
      moveit::planning_interface::MoveGroupInterface *_gripper_group;
};


class AGripperMoveSchunk: public BT::CoroActionNode
{
  public:
    AGripperMoveSchunk(const std::string& name, const BT::NodeConfiguration& config, 
    ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group)
    : BT::CoroActionNode(name,config),_nh(nh), _gripper_group(gripper_group)
    {
        _aborted = false;
    }

    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<std::string>("command")};
    } 

    private:
    std::string _commandin;
    bool _result; 
    bool _aborted; 
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_gripper_group;
};


inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

}

}

#endif