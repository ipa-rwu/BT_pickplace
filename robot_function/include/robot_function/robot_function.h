#ifndef ROBOT_FUNCTION_H_
#define ROBOT_FUNCTION_H_

#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/PoseStamped.h>


#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

#include <mutex>

#include <boost/bind.hpp>

struct pathplan
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success;
    /* data */
};

struct  gettarget
{
    geometry_msgs::Pose target_pose;
    bool success;
};


class RobotFunction
{
private:
    /* data */


    const robot_state::JointModelGroup* joint_model_group ;
    ros::Publisher planning_scene_diff_publisher;
    moveit_visual_tools::MoveItVisualTools *visual_tools;
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    const std::string _CameraTopicSub = "/camera/target/pose";

public:
    RobotFunction(){};
    RobotFunction(ros::NodeHandle nh);
    ~RobotFunction(){};


    // moveit::planning_interface::MoveGroupInterface *move_group;
    const std::string GROUP_MANIP = "manipulator";
    const std::string GROUP_GRIPP = "endeffector";
    ros::Subscriber camera_subscriber;
    bool TagGetTargetPose = false;
    geometry_msgs::Pose newTarget;


    void GetBasicInfo(moveit::planning_interface::MoveGroupInterface *move_group);
    // void InitialiseMoveit(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface move_group);
    void CameraCallback(const geometry_msgs::Pose::ConstPtr& camera_msg);
    bool comparePoses(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::Pose pose2, double delta_posistion=0.05, double delta_orientation=0.01);
    // void MoveToNamedTarget(std::string target);
    pathplan PathPlanning(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface *move_group);
    bool MoveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan);
    gettarget CameraFindTarget();
    gettarget KeepDistanceToTarget(geometry_msgs::Pose target_pose, double height);
    // bool CameraFindTarget();
    // bool MoveToPose(); 
};



#endif /* ROBOT_FUNCTION_H_ */
