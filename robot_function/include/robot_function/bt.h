#ifndef BT_NODES_H_
#define BT_NODES_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_function/robot_function.h"


inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}



// class BTWaitForTarget : public BT::SyncActionNode
// {
//   public:
//     BTWaitForTarget(const std::string& name, const BT::NodeConfiguration& config)
//     : BT::SyncActionNode(name, config)
//     {}    
//     BT::NodeStatus tick() override;

//     static BT::PortsList providedPorts()
//     { 
//       return { BT::OutputPort<geometry_msgs::Pose>("target")};
//     }


//   private:
//     bool _success;
//     ros::NodeHandle _nh;
//     int counter = 0;
// };

/*
class BTWaitForTarget : public BT::CoroActionNode
{
  public:
    BTWaitForTarget(const std::string& name):
        CoroActionNode(name, {})
    {}
    
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<geometry_msgs::Pose>("target")};
    };

    void halt() override;

    bool wasHalted() const { return _halted; }

  private:
    bool _halted;
    bool _success = false;
    int counter = 0;
     ros::NodeHandle _nh;
};
*/

class BTWaitForTarget : public BT::AsyncActionNode
{
  public:
    BTWaitForTarget(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
    : BT::AsyncActionNode(name, config), _nh(nh)
    {
      _aborted = false;
      _gettarget = false;
    }
    
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<geometry_msgs::Pose>("target")};
    };

    virtual void halt() override
    {
      _aborted = true;
    }

  private:
    bool _aborted;
    bool _gettarget;
    int _counter;
    RobotFunction robot_obj();
    ros::NodeHandle _nh;
};

/*
class BTPathPlanning : public BT::SyncActionNode
{
  public:
    BTPathPlanning(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::SyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _success = false;
    }

    BT::NodeStatus tick() override;

    // static BT::PortsList providedPorts() 
    // { 
    //   // printf("Target positions: [ %.1f, %.1f ]\n", _target.position.x, _target.position.y );
    //   return{ BT::InputPort<geometry_msgs::Pose>("target") };
    // }

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<geometry_msgs::Pose>("goal") };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

    private:
    bool _success;
    bool _aborted; 
    geometry_msgs::Pose _target;
    moveit::planning_interface::MoveGroupInterface::Plan _plan;
    int _counter;
    ros::NodeHandle _nh;
    RobotFunction _robot_obj;
    moveit::planning_interface::MoveGroupInterface *_move_group;

};
*/
class BTPathPlanning : public BT::AsyncActionNode
{
  public:
    BTPathPlanning(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::AsyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _success = false;
    }

    BT::NodeStatus tick() override;

    virtual void halt() override;

    // static BT::PortsList providedPorts() 
    // { 
    //   // printf("Target positions: [ %.1f, %.1f ]\n", _target.position.x, _target.position.y );
    //   return{ BT::InputPort<geometry_msgs::Pose>("target") };
    // }

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<geometry_msgs::Pose>("goal"),
         BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("makeplan")};
    } 

    private:
    bool _success;
    bool _aborted; 
    geometry_msgs::Pose _target;
    moveit::planning_interface::MoveGroupInterface::Plan _plan;
    int _counter;
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_move_group;
    // RobotFunction _robot_obj();
};

class BTFollowPath : public BT::AsyncActionNode
{
  public:
    BTFollowPath(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::AsyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _success = false;      
    }
    
    BT::NodeStatus tick() override;

    void halt() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<moveit::planning_interface::MoveGroupInterface::Plan>("planedplan") };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

  private:
    bool _success;
    bool _aborted; 
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_move_group;
    moveit::planning_interface::MoveGroupInterface::Plan _myplan;
};


inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

    // factory.registerNodeType<BTPathPlanning>("BTPathPlanning");

}

#endif /* BT_NODES_H_ */