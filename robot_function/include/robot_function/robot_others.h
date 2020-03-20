#ifndef ROBOT_OTHERS_H_
#define ROBOT_OTHERS_H_

#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>

#include <ar_marker_detector/getMarkerPose.h>
#include "robot_function/robot_function.h"


class OtherFunction
{
private:
    /* data */
    ros::ServiceClient _armarker_client;
    std::string _armarker_srv_name = "/getMarkerPose";
    ar_marker_detector::getMarkerPose _armarker_srv;



public:

    bool GetMarkerPose(ros::NodeHandle nh, int marker, geometry_msgs::Pose pose_temp);
};




#endif