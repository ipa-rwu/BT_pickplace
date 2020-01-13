#ifndef BT_NODES_H_
#define BT_NODES_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <moveit/move_group_interface/move_group_interface.h>

// class BTWaitForTarget : public BT::SyncActionNode
// {
//   public:
//     BTWaitForTarget(const std::string& name, const BT::NodeConfiguration& config)
//     : BT::SyncActionNode(name, config)
//     {}    
//     BT::NodeStatus tick() override;

//     static BT::PortsList providedPorts() { return {}; }


//   private:
//     bool _success;
//     ros::NodeHandle _nh;
//     int counter = 0;
// };
class BTWaitForTarget : public BT::CoroActionNode
{
  public:
    BTWaitForTarget(const std::string& name):
        CoroActionNode(name, {})
    {}
    
    BT::NodeStatus tick() override;

    void halt() override;

    bool wasHalted() const { return _halted; }

  private:
    bool _halted;
    bool _success = false;
    int counter = 0;
     ros::NodeHandle _nh;
};
class BTPathPlanning : public BT::SyncActionNode
{
  public:
    BTPathPlanning(const std::string& name, const BT::NodeConfiguration& config)
    : 
    BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() { return {}; }

    private:
    bool _success;
    moveit::planning_interface::MoveGroupInterface::Plan _plan;
    int counter = 0;
    ros::NodeHandle _nh;

};

class BTFollowPath : public BT::CoroActionNode
{
  public:
    BTFollowPath(const std::string& name, const BT::NodeConfiguration& config, bool& success)
    : BT::CoroActionNode(name, config), _success(success), _halted(false)
    {
    }
    
    BT::NodeStatus tick() override;

    void halt() override;

    bool wasHalted() const { return _halted; }

  private:
    bool _halted;
    ros::NodeHandle _nh;
    bool _success;
};


inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

    // factory.registerNodeType<BTPathPlanning>("BTPathPlanning");

}

#endif /* BT_NODES_H_ */
