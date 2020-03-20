#ifndef BTNODES_OTHERS_H_
#define BTNODES_OTHERS_H_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "robot_function/robot_others.h"


bool compare_pose(geometry_msgs::Pose A, geometry_msgs::Pose B, geometry_msgs::Pose C);

namespace BTNodesOthers
{

// AFindObjContainers
// input: blockmarker containermarkerA containermarkerB
// output: containerpose blockpose
class AFindObjContainers: public BT::CoroActionNode
{
  public:
    AFindObjContainers(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
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
            BT::InputPort<int>("blockmarker"),
            BT::InputPort<int>("containermarkerA"),
            BT::InputPort<int>("containermarkerB"),
            BT::OutputPort<geometry_msgs::Pose>("containerpose"),
            BT::OutputPort<geometry_msgs::Pose>("blockpose")
        };
    } 

  private:
    geometry_msgs::Pose _block_pose;
    geometry_msgs::Pose _container_pose_A;
    geometry_msgs::Pose _container_pose_B;
    bool _aborted;
    ros::NodeHandle _nh;
    int _container_marker_A;
    int _container_marker_B;
    int _block_marker;
};   

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{

}

}

#endif


/*
class AFindObjContainers: public BT::CoroActionNode
{
  public:
    AFindObjContainers(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle nh)
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
            BT::InputPort<int>("blockmarker"),
            BT::InputPort<int>("containermarkerA"),
            BT::InputPort<int>("containermarkerB"),
            BT::OutputPort<geometry_msgs::Pose>("containerpose"),
            BT::OutputPort<geometry_msgs::Pose>("blockpose")
        };
    } 

  private:
    gettarget _block;
    gettarget _container_A;
    gettarget _container_B;
    bool _aborted;
    ros::NodeHandle _nh;
    int _container_marker_A;
    int _container_marker_B;
    int _block_marker;
};   
*/