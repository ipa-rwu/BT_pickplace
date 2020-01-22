#include "ros/ros.h"

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>


class EnvironmentSet
{
    private:

    const double BASE_OFFSET_FROM_BACK_WALL_ = 0.28;   //28cm
    const double BASE_OFFSET_FROM_LEFT_WALL_ = 0.46;   //46cm
    const double BASE_OFFSET_FROM_RIGHT_WALL_ = 0.5;   //50cm
    const double TOTAL_INNER_CELL_Y_DIMENSION_ = 1.47; //1470cm
    const double TOTAL_INNER_CELL_X_DIMENSION_ = 0.96; //960cm
    const double TOTAL_INNER_CELL_Z_DIMENSION = 1.15;  //115cm
    ros::Publisher planning_scene_diff_publisher;

    public:

        void AddCollissionObjects(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group);

};