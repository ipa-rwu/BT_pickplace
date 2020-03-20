#ifndef BT_NODES_H_
#define BT_NODES_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_function/robot_function.h"
#include "robot_function/robot_gripper.h"
#include "robot_function/btnodes_arm.h"




class BTCheckCondition : public BT::SyncActionNode
{
  public:
    BTCheckCondition(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group) :
        BT::SyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _targetin.tag_name = false;
      _targetin.tag_pose = false;
      _targetin.tag_waypoint = false; 
      _targetin.Name = "none";
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
      static BT::PortsList providedPorts()
    {
      return { BT::InputPort<TargetType>("targetin"), BT::InputPort<double>("heightin")};
    };
  private:
    ros::NodeHandle _nh;
    TargetType _targetin;
    double _height;
    gettarget _subtarget;
    moveit::planning_interface::MoveGroupInterface *_move_group;

};

class BTWaitForTarget : public BT::AsyncActionNode
{
  public:
    BTWaitForTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
    {
      _aborted = false;
      _targetout.Waypoint.position.x = 0.0;
      _targetout.Waypoint.position.y = 0.0;
      _targetout.Waypoint.position.z = 0.0;
      _targetout.Waypoint.orientation.x = 0.0;
      _targetout.Waypoint.orientation.y = 0.0;
      _targetout.Waypoint.orientation.z = 0.0;
      _targetout.Waypoint.orientation.w = 0.0;
      _targetout.Pose.position.x = 0.0;
      _targetout.Pose.position.y = 0.0;
      _targetout.Pose.position.z = 0.0;
      _targetout.Pose.orientation.x = 0.0;
      _targetout.Pose.orientation.y = 0.0;
      _targetout.Pose.orientation.z = 0.0;
      _targetout.Pose.orientation.w = 0.0;
      _targetout.tag_name = false;
      _targetout.tag_pose = false;
      _targetout.tag_waypoint = false; 
      _targetout.Name = "none";

    }
    
    BT::NodeStatus tick() override;
    virtual void halt() override
    {
      _aborted = true;
    }

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<TargetType>("targetin"), BT::OutputPort<TargetType>("targetout")};
    };
   private:
    bool _aborted;
    TargetType _targetin;
    TargetType _targetout;
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
      _goal.tag_name = false;
      _goal.tag_pose = false;
      _goal.tag_waypoint = false; 
      _goal.Name = "none";
    }

    BT::NodeStatus tick() override;

    virtual void halt() override
    {
      _aborted = true;
    }

    // static BT::PortsList providedPorts() 
    // { 
    //   // printf("Target positions: [ %.1f, %.1f ]\n", _target.position.x, _target.position.y );
    //   return{ BT::InputPort<geometry_msgs::Pose>("target") };
    // }

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<TargetType>("goal"),
         BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("makeplan")};
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

/*
class BTFollowPath : public BT::AsyncActionNode
{
  public:
    BTFollowPath(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::AsyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _success = false; 
      _counter = 0;   
  
    }
    
    BT::NodeStatus tick() override;

    virtual void halt() override
    {
      _aborted = true;
    }

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
};



class BTFollowPath : public BT::SyncActionNode
{
  public:
    BTFollowPath(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::SyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _success = false; 
      _counter = 0;   
  
    }
    
    BT::NodeStatus tick() override;

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
};
*/

class BTFollowPath : public BT::SyncActionNode
{
  public:
    BTFollowPath(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::SyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _success = false; 
      _counter = 0;   
  
    }
    
    BT::NodeStatus tick() override;
  
  /*
    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }
    */

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
};



class BTCheckGripperCommand: public BT::CoroActionNode
{
  public:
    BTCheckGripperCommand(const std::string& name, const BT::NodeConfiguration& config)
    : BT::CoroActionNode(name,config)
    {
    }
    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<std::string>("commandin"), 
      BT::OutputPort<std::string>("commandout")};
    } 

    private:
    bool _aborted;
    std::string _commandin;
};

/*
class BTGripperMove: public BT::CoroActionNode
{
  public:
    BTGripperMove(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group)
    : BT::CoroActionNode(name,config),_nh(nh), _gripper_group(gripper_group)
    {
    }
    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<std::string>("commandin")};
    } 

    private:
    std::string _commandin;
    bool _aborted; 
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_gripper_group;
};
*/

class BTGripperMove: public BT::SyncActionNode
{
  public:
    BTGripperMove(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group)
    : BT::SyncActionNode(name,config),_nh(nh), _gripper_group(gripper_group)
    {
    }
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<std::string>("commandin")};
    } 

    private:
    std::string _commandin;
    bool _aborted; 
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_gripper_group;
};


class BTGripperMoveSchunk: public BT::SyncActionNode
{
  public:
    BTGripperMoveSchunk(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group)
    : BT::SyncActionNode(name,config),_nh(nh), _gripper_group(gripper_group)
    {
    }
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<std::string>("commandin")};
    } 

    private:
    std::string _commandin;
    bool _result; 
    ros::NodeHandle _nh;
    moveit::planning_interface::MoveGroupInterface *_gripper_group;
};

class BTCameraFindTarget : public BT::AsyncActionNode
{
  public:
    BTCameraFindTarget(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
    : BT::AsyncActionNode(name, config), _nh(nh)
    {
      _aborted = false;
      _gettarget = false;
      _target.tag_name = false;
      _target.tag_pose = false;
      _target.tag_waypoint = false; 
      _target.Name = "none";
    }
    
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      // return { BT::OutputPort<geometry_msgs::Pose>("targetout")};
      return { BT::OutputPort<TargetType>("targetout"), BT::InputPort<PositionGo>("targetin"),
                BT::OutputPort<bool>("tagisobjposeout")};
    };

    virtual void halt() override
    {
      _aborted = true;
    }

  private:
    bool _aborted;
    bool _gettarget;
    int _counter;
    TargetType _target;
    RobotFunction robot_obj();
    ros::NodeHandle _nh;
    gettarget _subtarget;


};


class BTIsObjPose : public BT::SyncActionNode
{
  public:
    BTIsObjPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
      _tagisobjpose = false;
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<TargetType>("targetin"),
               BT::InputPort<bool>("tagisobjposein"), 
               BT::OutputPort<TargetType>("targetout") };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

    private:
    bool _tagisobjpose;
    TargetType _objpose;
};

class BTStringtoPose : public BT::SyncActionNode
{
  public:
    BTStringtoPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<PositionGo>("stringin"),
               BT::OutputPort<geometry_msgs::Pose>("targetpose")};
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    }

  private:
      geometry_msgs::Pose _target_pose;
      PositionGo _goal;
};

class BTStringtoTarget : public BT::SyncActionNode
{
  public:
    BTStringtoTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
      _targetout.tag_name = false;
      _targetout.tag_pose = false;
      _targetout.tag_waypoint = false; 
      _targetout.Name = "none";
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<std::string>("string"),
               BT::OutputPort<TargetType>("targetout")};
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    }

  private:
      TargetType _targetout;
      std::string _stringin;
};

class BTStringToBool : public BT::SyncActionNode
{
  public:
    BTStringToBool(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
      _bool = false;
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{  BT::InputPort<std::string>("stringin"),
               BT::OutputPort<bool>("boolout")};
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    }

  private:
      bool _bool;
      std::string _string;
};


class BTIsHoldObj: public BT::SyncActionNode
{
  public:
    BTIsHoldObj(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name,config)
    {
      _tagishold = false;
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{ BT::InputPort<bool>("tagisholdin") };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

    private:
    bool _tagishold;
};

class BTIsObjContainer: public BT::SyncActionNode
{
  public:
    BTIsObjContainer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name,config)
    {
      _tagisobjcon = false;
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() 
    { 
      return{ BT::InputPort<bool>("tagisobjconin") };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

    private:
    bool _tagisobjcon;
};


/*
class BTIsHoldObj: public BT::CoroActionNode
{
  public:
    BTIsHoldObj(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
    : BT::CoroActionNode(name,config), _nh(nh)
    {
      _aborted = false;
    }
    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    private:
    bool _aborted;
    RobotFunction robot_obj();
    ros::NodeHandle _nh;

};
*/

class BTCloseToTarget: public BT::AsyncActionNode
{
  public:
    BTCloseToTarget(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh, 
    moveit::planning_interface::MoveGroupInterface *move_group)
    : BT::AsyncActionNode(name, config), _nh(nh), _move_group(move_group)
    {
      _aborted = false;
      _gettarget = false;
      _counter = 0;   
      _targetout.tag_name = false;
      _targetout.tag_pose = false;
      _targetout.tag_waypoint = false;
      _targetout.Name = "none";   
    }
    
    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<TargetType>("targetin"), BT::InputPort<double>("height"), 
      BT::OutputPort<TargetType>("targetout"), BT::InputPort<std::string>("targettype")};
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
    TargetType _obstarget;
    TargetType _targetout;
    geometry_msgs::Pose _goal;
    double _height;
    gettarget _subtarget;
    ros::NodeHandle _nh;
    std::string _targettype;
    moveit::planning_interface::MoveGroupInterface *_move_group;
};

class BTAdvertiseGripperCommand: public BT::SyncActionNode
{
  public:
    BTAdvertiseGripperCommand(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override;

    // static BT::PortsList providedPorts() 
    // { 
    //   // printf("Target positions: [ %.1f, %.1f ]\n", _target.position.x, _target.position.y );
    //   return{ BT::InputPort<geometry_msgs::Pose>("target") };
    // }

    static BT::PortsList providedPorts() 
    { 
      return{  BT::OutputPort<std::string>("commandout"),
      BT::InputPort<std::string>("commandin") };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

    private:
    std::string _command;

}; 

class BTPubFakeHoldObj: public BT::CoroActionNode
{
  public:
    BTPubFakeHoldObj(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
    : BT::CoroActionNode(name,config), _nh(nh)
    {
      _aborted = false;
    }
    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    private:
    bool _aborted;
    RobotFunction robot_obj();
    ros::NodeHandle _nh;
};

/*
// TODO genertate from configuration
class AReloadParamArm: public BT::CoroActionNode
{
  public:
    AReloadParamArm(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
    : BT::CoroActionNode(name,config), _nh(nh)
    {
      _aborted = false;
    }
    BT::NodeStatus tick() override;

    void halt() override
    {
      _aborted = true;
      BT::CoroActionNode::halt();
    }

    // TODO: generate from configuration
    static BT::PortsList providedPorts() 
    { 
      return{ 
        BT::OutputPort<ParamType>("param"),
         
        // BT::OutputPort<double>("T2S1"),
        // BT::OutputPort<double>("T2S2"),
        // BT::OutputPort<double>("T2S3"),
        // BT::OutputPort<double>("T3S1"),
        // BT::OutputPort<double>("T3S2"),
        // BT::OutputPort<double>("T3S3"),
        BT::InputPort<bool>("bool") };
      // BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan"),
    } 

    private:
    bool _aborted;
    bool _first;
    ParamClient _paramcli_obj;
    ParamType _param;
    ros::NodeHandle _nh;
    static const int _size = 6;
    double _param_arm_temp[_size] = {0.00};
    const std::string _port_name[_size] = {"T2S1", "T2S2", "T2S3", "T3S1", "T3S2", "T3S3"};
};
*/

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

    // factory.registerNodeType<BTPathPlanning>("BTPathPlanning");

}

#endif /* BT_NODES_H_ */
