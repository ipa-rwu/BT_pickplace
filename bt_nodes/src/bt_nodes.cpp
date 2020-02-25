#include "bt_nodes.h"


BT_REGISTER_NODES(factory)
{
    RobotControl::RegisterNodes(factory);
}

namespace RobotControl
{

BT::NodeStatus BTPathPlanning::tick()
{
    counter++;
    std::cout<<"BTPathPlanning: "<< counter <<std::endl;
    if(_success)
    {
      std::cout << "PathPlanning: SUCCESS" << std::endl;
      setOutput("pathplan", _plan);
      return BT::NodeStatus::SUCCESS;  
    }
    else
    {
      return BT::NodeStatus::FAILURE;  
    }
    
}

//execute
BT::NodeStatus BTFollowPath::tick()
{
    //  auto res = getInput<moveit::planning_interface::MoveGroupInterface::Plan>("planedpath");
    // if( !res )
    // {
    //     throw RuntimeError("error reading port [planedpath]:", res.error() );
    // }
    // else
    // {
    //   _plan = res.value;
    // } 
    _halted = false;  
    while (!_success)
    {
      setStatusRunningAndYield();

    }
    std::cout << "ExecutePlan: SUCCESS"<< std::endl;
    return BT::NodeStatus::SUCCESS;
    // if(_success)
    // {
    //   std::cout << "ExecutePlan: " << _success << std::endl;
    //   return BT::NodeStatus::SUCCESS;  
    // }
    // else
    // {
    //   return BT::NodeStatus::FAILURE;  
    // }

}

void BTFollowPath::halt() 
{
    std::cout << "ExecutePlan::halt" << std::endl;
    _halted = true;
    CoroActionNode::halt();
}

//camera
BT::NodeStatus BTWaitForTarget::tick()
{
    counter++;
    std::cout<<"print "<< counter <<std::endl;
    std::cout<<"print "<< _success <<std::endl;

    if(_success)
    {
    std::cout << "BTWaitForTarget: SUCCESS"<< std::endl;
    return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;  
    }
}



//Griiper
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