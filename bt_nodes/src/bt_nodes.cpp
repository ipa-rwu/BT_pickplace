#include "bt_nodes.h"


BT_REGISTER_NODES(factory)
{
    RobotControl::RegisterNodes(factory);
}

namespace RobotControl
{

BT::NodeStatus BTPathPlanning::tick()
{
    if(_success)
    {
      std::cout << "PathPlanning: " << _success << std::endl;
      setOutput("pathplan", _plan);
      return BT::NodeStatus::SUCCESS;  
    }
    else
    {
      return BT::NodeStatus::FAILURE;  
    }
    
}


BT::NodeStatus BTFollowPath::tick()
{
     auto res = getInput<moveit::planning_interface::MoveGroupInterface::Plan>("planedpath");
    // if( !res )
    // {
    //     throw RuntimeError("error reading port [planedpath]:", res.error() );
    // }
    // else
    // {
    //   _plan = res.value;
    // }   
    if(_success)
    {
      std::cout << "ExecutePlan: " << _success << std::endl;
      return BT::NodeStatus::SUCCESS;  
    }
    else
    {
      return BT::NodeStatus::FAILURE;  
    }

}

void BTFollowPath::halt() 
{
    std::cout << "ExecutePlan::halt" << std::endl;
    _halted = true;
    CoroActionNode::halt();
}

BT::NodeStatus GripperInterface::open()
{
    _opened = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close()
{
    std::cout << "GripperInterface::close" << std::endl;
    _opened = false;
    return BT::NodeStatus::SUCCESS;
}



}