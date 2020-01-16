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

class BTCheckCondition : public BT::SyncActionNode
{
  public:
    BTCheckCondition(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group) :
        BT::SyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
      static BT::PortsList providedPorts()
    {
      return { BT::InputPort<geometry_msgs::Pose>("targetin"), BT::InputPort<double>("heightin")};
    };
  private:
    ros::NodeHandle _nh;
    geometry_msgs::Pose _obstarget;
    double _height;
    moveit::planning_interface::MoveGroupInterface *_move_group;

};

class BTWaitForTarget : public BT::AsyncActionNode
{
  public:
    BTWaitForTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
    {
      _aborted = false;
      _gettarget = false;
      _target.position.x = 0.0;
      _target.position.y = 0.0;
      _target.position.z = 0.0;
      _target.orientation.x = 0.0;
      _target.orientation.y = 0.0;
      _target.orientation.z = 0.0;
      _target.orientation.w = 0.0;
      _pretarget = _target;
      _statewaitfortarget = true;
      _stateexecutefree = true;

    }
    
    BT::NodeStatus tick() override;
        virtual void halt() override
    {
      _aborted = true;
    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<geometry_msgs::Pose>("targetin"), BT::OutputPort<geometry_msgs::Pose>("targetout"),
      BT::OutputPort<bool>("statewaittarget"),
      BT::OutputPort<bool>("stateexecutefree")};
    };
   private:
    bool _aborted;
    bool _gettarget;
    int _counter;
    geometry_msgs::Pose _target;
    geometry_msgs::Pose _pretarget;
    bool _success;
    bool _statewaitfortarget;
    bool _stateexecutefree;
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
      _counter = 0;   
      _statewaitfortarget = false;
      _stateexecutefree = false;
  
    }
    
    BT::NodeStatus tick() override;

    void halt() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<moveit::planning_interface::MoveGroupInterface::Plan>("planedplan"), 
      BT::OutputPort<bool>("statewaittarget"),
      BT::OutputPort<bool>("stateexecutefree")};
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

  private:
    bool _success;
    bool _aborted; 
    ros::NodeHandle _nh;
    int _counter;
    moveit::planning_interface::MoveGroupInterface *_move_group;
    moveit::planning_interface::MoveGroupInterface::Plan _myplan;
    bool  _statewaitfortarget;
    bool _stateexecutefree;
};



class BTCameraFindTarget : public BT::AsyncActionNode
{
  public:
    BTCameraFindTarget(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
    : BT::AsyncActionNode(name, config), _nh(nh)
    {
      _aborted = false;
      _gettarget = false;
    }
    
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<geometry_msgs::Pose>("targetout")};
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
        bool _firsttime;

};

class BTCloseToTarget : public BT::AsyncActionNode
{
  public:
    BTCloseToTarget(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, 
    moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::AsyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _gettarget = false;
      _counter = 0;      
    }
    
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<geometry_msgs::Pose>("targetin"), BT::InputPort<double>("height"), 
      BT::OutputPort<geometry_msgs::Pose>("targetout"), 
      BT::InputPort<bool>("statewaittarget"),
      BT::InputPort<bool>("stateexecutefree")};
    };

    virtual void halt() override
    {
      _aborted = true;
      std::cout << "[ BTCloseToTarget: success out]" << std::endl;
    }

  private:
    bool _aborted;
    bool _gettarget;
    int _counter;
    RobotFunction robot_obj();
    geometry_msgs::Pose _obstarget;
    geometry_msgs::Pose _pretarget;
    double _height;
    double _preheight;
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_move_group;
    bool _statewaitfortarget;
    bool _stateexecutefree;
};



inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

    // factory.registerNodeType<BTPathPlanning>("BTPathPlanning");

}

#endif /* BT_NODES_H_ */
