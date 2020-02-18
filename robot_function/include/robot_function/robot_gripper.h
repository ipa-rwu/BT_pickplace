#ifndef ROBOT_GRIPPER_H_
#define ROBOT_GRIPPER_H_

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


class GripperFunction
{
private:
    /* data */

    const robot_state::JointModelGroup* gripper_model_group ;
    ros::Publisher planning_scene_diff_publisher;
    moveit_visual_tools::MoveItVisualTools *visual_tools;
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

    const int IO_SERVICE_FUN_LEVEL_ = 1;   // Not exactly sure what this is, but 1 seems to work. If it fails, try 2.

public:
    // GripperFunction(){};
    // GripperFunction(ros::NodeHandle nh);
    // ~GripperFunction(){};


    // moveit::planning_interface::MoveGroupInterface *move_group;
    const std::string GROUP_GRIPP = "gripper";
    // ros::Subscriber camera_subscriber;

    void InitialiseGripper(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *gripper_group);

    // void CameraCallback(const geometry_msgs::Pose::ConstPtr& camera_msg);
    bool MoveGripper(moveit::planning_interface::MoveGroupInterface *gripper_group, std::string target);

    //schunk 
    bool GripperOpen(ros::NodeHandle nh);
    bool GripperClose(ros::NodeHandle nh);


};



#endif /* ROBOT_GRIPPER_H_ */
