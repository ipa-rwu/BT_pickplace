#include "robot_function/environment.h"


void EnvironmentSet::AddCollissionObjects(ros::NodeHandle nh, moveit::planning_interface::MoveGroupInterface *move_group)
{
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  moveit_msgs::AttachedCollisionObject object;
  object.link_name = move_group->getPlanningFrame();
  object.object.header.frame_id = move_group->getPlanningFrame();
  object.object.id = "Floor";


  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;  // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_; // Base is ofset by (0.1470/2-.275)
  box_pose.position.z = -0.01; //Push it slightly down to avoid collission with base plate.

  // Since we are attaching the object to the robot base
  // we want the collision checker to ignore collisions between the object and the robot base
  object.touch_links = std::vector<std::string>{ "base_link"};
  moveit_msgs::PlanningScene planning_scene;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);


  // The id of the object is used to identify it.
  object.object.id = "Cieling";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;  // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_; // Base is ofset by (0.1470/2-.275)
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

   // The id of the object is used to identify it.
  object.object.id  = "Left Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = BASE_OFFSET_FROM_RIGHT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Right Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -BASE_OFFSET_FROM_LEFT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Back Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = 0;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;
  box_pose.position.y = -BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO_NAMED("kogrob", "Adding collission objects into the world");

}

