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
      return BT::NodeStatus::SUCCESS;  
    }
    else
    {
      return BT::NodeStatus::FAILURE;  
    }
    
}



}