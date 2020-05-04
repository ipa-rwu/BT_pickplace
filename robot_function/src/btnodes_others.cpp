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

  if( !getInput<int>("blockmarker", _block_marker))
  {
    throw BT::RuntimeError("AFindObjContainers missing required input [blockmarker]");
  }


  // if( !getInput<geometry_msgs::Pose>("containerposeA", _container_pose_A))
  if( !getInput<TargetType>("containerposeA", _container_pose_A))

  {
    throw BT::RuntimeError("AFindObjContainers missing required input [containermarkerA]");
  }

    // if( !getInput<geometry_msgs::Pose>("containerposeB", _container_pose_B))
  if( !getInput<TargetType>("containerposeB", _container_pose_B))
  {
    throw BT::RuntimeError("AFindObjContainers missing required input [containermarkerB]");
  }


  while(!_aborted)
  {

    if ( other_obj.GetMarkerPose(_nh, _block_marker, _block_pose) )
    // && other_obj.GetMarkerPose(_nh, _container_marker_A, _container_pose_A) &&
    //     other_obj.GetMarkerPose(_nh, _container_marker_B, _container_pose_B) 
        {
          // _block_pose.orientation.z = 0.707;
          // _block_pose.orientation.w = 0.707;
          

            setOutput<geometry_msgs::Pose>("blockpose", _block_pose);
            printf("AFindObjContainers got blockpose : x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
             _block_pose.position.x, _block_pose.position.y, _block_pose.position.z,
             _block_pose.orientation.x, _block_pose.orientation.y, _block_pose.orientation.z, 
             _block_pose.orientation.w);

            // A > B
            if ( compare_pose (_block_pose, _container_pose_A.Pose, _container_pose_B.Pose))
            {
                printf("AFindObjContainers got containerpose A: x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
                _container_pose_A.Pose.position.x, _container_pose_A.Pose.position.y, _container_pose_A.Pose.position.z,
                _container_pose_A.Pose.orientation.x, _container_pose_A.Pose.orientation.y, _container_pose_A.Pose.orientation.z, 
                _container_pose_A.Pose.orientation.w);
                setOutput<geometry_msgs::Pose>("containerpose", _container_pose_A.Pose);
                return BT::NodeStatus::SUCCESS;
            }  

            else
            {
              printf("AFindObjContainers got containerpose B: x=%.4f y=%.4f z=%.4f ox=%.4f oy=%.4f oz=%.4f ow=%.4f\n",
              _container_pose_B.Pose.position.x, _container_pose_B.Pose.position.y, _container_pose_B.Pose.position.z,
              _container_pose_B.Pose.orientation.x, _container_pose_B.Pose.orientation.y, _container_pose_B.Pose.orientation.z, 
              _container_pose_B.Pose.orientation.w);
                setOutput<geometry_msgs::Pose>("containerpose", _container_pose_B.Pose);
                return BT::NodeStatus::SUCCESS;
             }
          
        }
  }

  return BT::NodeStatus::FAILURE;
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