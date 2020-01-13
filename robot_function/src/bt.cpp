#include "robot_function/bt.h"
#include "robot_function/robot_function.h"

//camera
// BT::NodeStatus BTWaitForTarget::tick()
// {
//   RobotFunction robot_obj(_nh);
//   gettarget subtarget = robot_obj.CameraFindTarget();
//   std::cout << "BTWaitForTarget: "<< subtarget.success << std::endl;
  
//   if(_success = subtarget.success)
//     {
//     std::cout << "BTWaitForTarget: SUCCESS"<< std::endl;
//     setOutput<geometry_msgs::Pose>("target", subtarget.target_pose);
//     return BT::NodeStatus::SUCCESS;
//     }
//   return BT::NodeStatus::FAILURE;  
    
// }

// BT::NodeStatus BTWaitForTarget::tick()
// {
//   RobotFunction robot_obj(_nh);
//   bool subtarget = robot_obj.CameraFindTarget();
//   std::cout << "BTWaitForTarget: "<<subtarget<< std::endl;
  
//   if(_success = subtarget)
//     {
//     std::cout << "BTWaitForTarget: SUCCESS"<< std::endl;
//     // setOutput<geometry_msgs::Pose>("target", subtarget.target_pose);
//     return BT::NodeStatus::SUCCESS;
//     }
//   return BT::NodeStatus::FAILURE;  
    
// }

BT::NodeStatus BTWaitForTarget::tick()
{
  RobotFunction robot_obj(_nh);
    counter++;
    std::cout<<"print "<< counter <<std::endl;
    _halted = false;  
    gettarget subtarget;
    subtarget.success = false;
    while (!subtarget.success)
    {
      subtarget = robot_obj.CameraFindTarget();

      if(!subtarget.success)
      {
        setStatusRunningAndYield();

      }

    }
    std::cout << "FindTarget: SUCCESS"<< std::endl;
    setOutput<geometry_msgs::Pose>("target", subtarget.target_pose);

    return BT::NodeStatus::SUCCESS;
}
void BTWaitForTarget::halt() 
{
    std::cout << "ExecutePlan::halt" << std::endl;
    _halted = true;
    CoroActionNode::halt();
}

BT::NodeStatus BTPathPlanning::tick()
{
  RobotFunction robot_obj(_nh);
  auto res = getInput<geometry_msgs::Pose>("target");
  if( !res )
  {
    // no target go back to wait for target
    return BT::NodeStatus::FAILURE;  
  }
  pathplan planpath = robot_obj.PathPlanning(res.value());
  if(_success = planpath.success)
  {
    std::cout << "PathPlanning: SUCCESS" << std::endl;
    setOutput("pathplan", planpath.plan);
    return BT::NodeStatus::SUCCESS;  
  }
  return BT::NodeStatus::FAILURE;  
    
}