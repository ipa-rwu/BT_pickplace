
#ifndef BT_NODES_H_
#define BT_NODES_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


class TestNode
{
    public:
    TestNode(const std::string& name);
};

class BTPathPlanning : public BT::SyncActionNode, public TestNode
{
  public:
    BTPathPlanning(const std::string& name)
    : BT::SyncActionNode(name, {}), TestNode(name)
    {}

    BT::NodeStatus tick() override;
};


class BTFollowPath : public BT::CoroActionNode, public TestNode
{
  public:
    BTFollowPath(const std::string& name)
    : BT::CoroActionNode(name, {}), TestNode(name), _halted(false)
    {}

    void halt() override;

  private:
     bool _halted;

};

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
