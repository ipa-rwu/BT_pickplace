#include "robot_function/robot_others.h"

bool OtherFunction::GetMarkerPose(ros::NodeHandle nh, int marker, geometry_msgs::Pose &pose_temp)
{
    pose_temp.position.x = 0;
    pose_temp.position.y = 0;
    pose_temp.position.z = 0; 
    pose_temp.orientation.x = 0;
    pose_temp.orientation.y = 0; 
    pose_temp.orientation.z = 0; 
    pose_temp.orientation.w = 0;

    ar_marker_detector::getMarkerPose _armarker_srv;

    ros::ServiceClient _armarker_client;

    _armarker_client = nh.serviceClient<ar_marker_detector::getMarkerPose>(_armarker_srv_name);

    _armarker_srv.request.ar_marker_id = marker;
    
    // printf("OtherFunction GetMarkerPose : marker =%d ",
    //  _armarker_srv.request.ar_marker_id);  
    if(_armarker_client.call(_armarker_srv))
    {
        pose_temp = _armarker_srv.response.pose;
        // pose_private = _armarker_srv.response.pose;
        if (pose_temp.position.x == 0 && pose_temp.position.y == 0 &&
            pose_temp.position.z == 0 && pose_temp.orientation.x == 0 &&
            pose_temp.orientation.y == 0 && pose_temp.orientation.z == 0 &&
            pose_temp.orientation.w == 0)
            {
                return false;
            }
        else
        {
                printf("OtherFunction GetMarkerPose : marker =%d ",
                  _armarker_srv.request.ar_marker_id);  
                printf("OtherFunction got target : x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
                 pose_temp.position.x, pose_temp.position.y, pose_temp.position.z,
                 pose_temp.orientation.x, pose_temp.orientation.y, pose_temp.orientation.z, 
                 pose_temp.orientation.w);
                // printf("GetMarkerPose got target : x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
                //  pose_private.position.x, pose_private.position.y, pose_private.position.z,
                //  pose_private.orientation.x, pose_private.orientation.y, pose_private.orientation.z, 
                //  pose_private.orientation.w);

                return true;
            }
        
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