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

class RobotFunction
{
private:
    /* data */


    const robot_state::JointModelGroup* joint_model_group ;
    ros::Publisher planning_scene_diff_publisher;
    moveit_visual_tools::MoveItVisualTools *visual_tools;
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

public:
    // RobotFunction(/* args */);
    // ~RobotFunction();

    moveit::planning_interface::MoveGroupInterface *move_group;
    const std::string GROUP_MANIP = "manipulator";
    const std::string GROUP_GRIPP = "endeffector";


    void GetBasicInfo();
    void InitialiseMoveit(ros::NodeHandle nh);
    // void MoveToNamedTarget(std::string target);
    pathplan PathPlanning(geometry_msgs::Pose target_pose);
    bool MoveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan plan);
    bool PlanExecute(geometry_msgs::Pose target_pose);

    // bool MoveToPose(); 
};



#endif /* ROBOT_FUNCTION_H_ */
