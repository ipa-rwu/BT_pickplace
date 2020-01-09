
#ifndef BT_NODES_H_
#define BT_NODES_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"



namespace RobotControl
{

//   class BTPathPlanning : public BT::SyncActionNode
// {
//   public:
//     BTPathPlanning(const std::string& name, const BT::NodeConfiguration& config)
//     : 
//     BT::SyncActionNode(name, config)
//     {}

//     BT::NodeStatus tick() override;

//     static BT::PortsList providedPorts() { return {}; }

//     private:
//     bool _success;
// };

class BTPathPlanning : public BT::SyncActionNode
{
  public:
    BTPathPlanning(const std::string& name, const BT::NodeConfiguration& config, bool success)
    : 
    BT::SyncActionNode(name, config), _success(success)
    {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() { return {}; }

    private:
    bool _success;
};


// class BTFollowPath : public BT::CoroActionNode, public TestNode
// {
//   public:
//     BTFollowPath(const std::string& name)
//     : BT::CoroActionNode(name, {}), TestNode(name), _halted(false)
//     {}

//     void halt() override;

//   private:
//      bool _halted;

// };

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
