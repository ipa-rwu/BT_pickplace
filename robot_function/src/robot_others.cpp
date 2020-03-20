#include "robot_function/robot_others.h"

bool OtherFunction::GetMarkerPose(ros::NodeHandle nh, int marker, geometry_msgs::Pose pose_temp)
{
    _armarker_client = nh.serviceClient<ar_marker_detector::getMarkerPose>(_armarker_srv_name);
    _armarker_srv.request.ar_marker_id = marker;
    if(_armarker_client.call(_armarker_srv))
    {
        pose_temp = _armarker_srv.response.pose;
        return true;
    }
    return false;
}

/*
gettarget OtherFunction::GetMarkerPose(ros::NodeHandle nh, int marker)
{
    gettarget result;
    _armarker_client = nh.serviceClient<ar_marker_detector::getMarkerPose>(_armarker_srv_name);
    _armarker_srv.request.ar_marker_id = marker;
    if(_armarker_client.call(_armarker_srv))
    {
        result.target_pose = _armarker_srv.response.pose;
        result.success = true;
        return result;
    }
    result.success = false;
    return result;
}
*/