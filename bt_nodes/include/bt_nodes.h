
#ifndef BT_NODES_H_
#define BT_NODES_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <moveit/move_group_interface/move_group_interface.h>


namespace RobotControl
{

class BTPathPlanning : public BT::SyncActionNode
{
  public:
    BTPathPlanning(const std::string& name, const BT::NodeConfiguration& config, bool success, moveit::planning_interface::MoveGroupInterface::Plan plan)
    : 
    BT::SyncActionNode(name, config), _success(success), _plan(plan)
    {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() { return {}; }

    private:
    bool _success;
    moveit::planning_interface::MoveGroupInterface::Plan _plan;
};

class GripperInterface
{

  public:
    GripperInterface() : _opened(true)
    {
    }

    BT::NodeStatus open();

    BT::NodeStatus close();

  private:
    bool _opened;
};


class BTFollowPath : public BT::CoroActionNode
{
  public:
    BTFollowPath(const std::string& name, const BT::NodeConfiguration& config, bool success)
    : BT::CoroActionNode(name, config), _success(success), _halted(false)
    {}
    
    BT::NodeStatus tick() override;

    void halt() override;

    bool wasHalted() const { return _halted; }

  private:
    bool _halted;
    bool _success;

};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

    // factory.registerNodeType<BTPathPlanning>("BTPathPlanning");

}

}

// class BTPlanningInterface
// {
//     public:
//     BTPlanningInterface();
//     BTPlanningInterface(int unkonw);
//     ~BTPlanningInterface();

//     BT::NodeStatus open();
    
//     private:
// };

// class BTMovingInterface
// {
//     public:
//     BT::NodeStatus move();
//     private:
// };


#endif /* BT_NODES_H_ */
