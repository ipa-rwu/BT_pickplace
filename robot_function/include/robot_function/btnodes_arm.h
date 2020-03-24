#ifndef BTNODES_ARM_H_
#define BTNODES_ARM_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_function/robot_function.h"
#include "robot_function/btnodes_param.h"


struct PositionGo
{
  double px,py,pz,ox,oy,oz,ow;
};

struct TargetType
{
  std::string Name;
  bool tag_name;
  geometry_msgs::Pose Pose;
  bool tag_pose; 
  geometry_msgs::Pose Waypoint;
  bool tag_waypoint;
};

namespace BT
{	
template <> inline TargetType convertFromString(StringView str)
{
		// printf("Converting string: \"%s\"\n", str.data() );

		// real numbers separated by semicolons
		auto parts = splitString(str, ';');
    TargetType target;
    target.tag_name = false;
    target.tag_pose = false;
    target.tag_waypoint = false;
    target.Name = "none";

    if (convertFromString<std::string>(parts[0]) == "name")
    {
      target.tag_name = true;
      target.Name = convertFromString<std::string>(parts[1]);
      return target;
    }
    if (convertFromString<std::string>(parts[0]) == "pose")
    {
      target.tag_pose = true;
      target.Pose.position.x = convertFromString<double>(parts[1]);
      target.Pose.position.y = convertFromString<double>(parts[2]);
      target.Pose.position.z = convertFromString<double>(parts[3]);
      target.Pose.orientation.x = convertFromString<double>(parts[4]);
      target.Pose.orientation.y = convertFromString<double>(parts[5]);
      target.Pose.orientation.z = convertFromString<double>(parts[6]);
      target.Pose.orientation.w = convertFromString<double>(parts[7]);
      return target;
    }

    if (convertFromString<std::string>(parts[0]) == "waypoint")
    {
      target.tag_waypoint = true;
      target.Waypoint.position.x = convertFromString<double>(parts[1]);
      target.Waypoint.position.y = convertFromString<double>(parts[2]);
      target.Waypoint.position.z = convertFromString<double>(parts[3]);
      target.Waypoint.orientation.x = convertFromString<double>(parts[4]);
      target.Waypoint.orientation.y = convertFromString<double>(parts[5]);
      target.Waypoint.orientation.z = convertFromString<double>(parts[6]);
      target.Waypoint.orientation.w = convertFromString<double>(parts[7]);
      return target;
    }
}
}


namespace BTNodesArm
{

// Pathplanning: 
// input: goal
// output: plan 
class APathPlanning : public BT::CoroActionNode
{
  public:
    APathPlanning(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, 
    moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::CoroActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _success = false;
      _goal.tag_name = false;
      _goal.tag_pose = false;
      _goal.tag_waypoint = false; 
      _goal.Name = "none";
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
        BT::InputPort<TargetType>("goal"),
        BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("plan")};
    } 

    private:
    bool _success;
    bool _aborted; 
    TargetType _goal;
    moveit::planning_interface::MoveGroupInterface::Plan _plan;
    pathplan _planpath;
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_move_group;
    const double _jump_threshold = 0.0;
    const double _eef_step = 0.01;
};

// AFollowPath
//  input: plan
class AFollowPath : public BT::CoroActionNode
{
  public:
    AFollowPath(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::CoroActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _success = false; 
      _counter = 0;   
  
    }
    
    BT::NodeStatus tick() override;
  
    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    static BT::PortsList providedPorts() 
    { 
      return
      {  
        BT::InputPort<moveit::planning_interface::MoveGroupInterface::Plan>("plan"), 
      };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

  private:
    bool _success;
    bool _aborted; 
    ros::NodeHandle _nh;
    int _counter;
    moveit::planning_interface::MoveGroupInterface *_move_group;
    moveit::planning_interface::MoveGroupInterface::Plan _myplan;
};

// APreparePoseArm:
// input:  targetin, param(height), targettype(waypoint), step(1)
// output: targetout (pose to go)
class APreparePoseArm: public BT::CoroActionNode
{
  public:
    APreparePoseArm(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, 
    moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::CoroActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _gettarget = false;
      _step = 0;   
      _targetout.tag_name = false;
      _targetout.tag_pose = false;
      _targetout.tag_waypoint = false;
      _targetout.Name = "none";   

      // _param.arm.param[6] = {0.0000};
      // _param.gripper.param[6] = {"none"};
      // _param.flag.param[4] = {false};
    }
    
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return 
      { 
          BT::InputPort<geometry_msgs::Pose>("goalarm"), 
          BT::InputPort<ParamType>("param"), 
          BT::InputPort<std::string>("targettype"), 
          BT::InputPort<int>("step"),
          BT::OutputPort<TargetType>("targetout")};
    };

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

  private:
    bool _aborted;
    bool _gettarget;
    int _step;
    TargetType _targetout;
    geometry_msgs::Pose  _goal;
    ParamType _param;
    gettarget _subtarget;
    ros::NodeHandle _nh;
    std::string _targettype;
    moveit::planning_interface::MoveGroupInterface *_move_group;
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

    // factory.registerNodeType<BTPathPlanning>("BTPathPlanning");

}

}

#endif