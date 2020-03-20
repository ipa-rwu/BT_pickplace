#include "robot_function/btnodes_others.h"

BT_REGISTER_NODES(factory)
{
    BTNodesOthers::RegisterNodes(factory);
}


bool compare_pose(geometry_msgs::Pose A, geometry_msgs::Pose B, geometry_msgs::Pose C)
{
    double AB =
    ( A.position.x-B.position.x ) * ( A.position.x-B.position.x ) +
    ( A.position.y-B.position.y ) * ( A.position.y-B.position.y ) +
    ( A.position.z-B.position.z ) * ( A.position.z-B.position.z );

    double AC =
    ( A.position.x-C.position.x ) * ( A.position.x-C.position.x ) +
    ( A.position.y-C.position.y ) * ( A.position.y-C.position.y ) +
    ( A.position.z-C.position.z ) * ( A.position.z-C.position.z );

    if (AB > AC)
    {
        return true;
    }

    return false; 
}


namespace BTNodesOthers
{

BT::NodeStatus AFindObjContainers::tick()
{
  OtherFunction other_obj;

  if( !getInput<int>("blockmarker", _block_marker) ||
   !getInput<int>("containermarkerA", _container_marker_A) || 
   !getInput<int>("containermarkerB", _container_marker_B))
  {
    throw BT::RuntimeError("AFindObjContainers missing required input [marker]");
  }

  while(!_aborted)
  {
    if ( other_obj.GetMarkerPose(_nh, _block_marker, _block_pose) && 
        other_obj.GetMarkerPose(_nh, _container_marker_A, _container_pose_A) &&
        other_obj.GetMarkerPose(_nh, _container_marker_B, _container_pose_B) )
        {
            setOutput<geometry_msgs::Pose>("blockpose", _block_pose);
            // A > B
            if ( compare_pose (_block_pose, _container_pose_A, _container_pose_B))
            {
                setOutput<geometry_msgs::Pose>("container", _container_pose_A);
                return BT::NodeStatus::SUCCESS;
            }  

            else
            {
                setOutput<geometry_msgs::Pose>("containerpose", _container_pose_B);
                return BT::NodeStatus::SUCCESS;
             }
          
        }
  }

  if ( _aborted)
  {
    return BT::NodeStatus::FAILURE;
  }
}

}

/*
  while(!_aborted)
  {
      _block = other_obj.GetMarkerPose(_nh, _block_marker);
      _container_A = other_obj.GetMarkerPose(_nh, _container_marker_A);
      _container_B = other_obj.GetMarkerPose(_nh, _container_marker_B);

    if ( _block.success && 
        _container_A.success &&
        _container_B.success )
        {
            setOutput<geometry_msgs::Pose>("blockpose", _block.target_pose);
            // A > B
            if ( compare_pose (_block.target_pose, _container_A.target_pose, _container_B.target_pose))
            {
                setOutput<geometry_msgs::Pose>("container", _container_A.target_pose);
                return BT::NodeStatus::SUCCESS;
            }  

            else
            {
                setOutput<geometry_msgs::Pose>("containerpose", _container_B.target_pose);
                return BT::NodeStatus::SUCCESS;
             }
          
        }
  }
*/