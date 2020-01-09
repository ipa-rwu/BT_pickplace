#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include <unistd.h>
#include <stdio.h>

#include "ros/ros.h"
#include "robot_function.h"
#include "bt_nodes.h"

using namespace BT;

int main(int argc, char **argv)
{
    /*
    // print current path
    char cwd[1024];
    chdir("/path/to/change/directory/to");
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);
    */

    ros::init(argc, argv, "kogrob_demo");
    ros::NodeHandle nh;
    
    // robot function
    RobotFunction robot_obj;

    // behavior tree nodes
    // using namespace RobotControl;
    BehaviorTreeFactory factory;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string xml_filename;
    nh.param<std::string>("file", xml_filename, "src/robot_function/treexml/test_tree.xml");
    ROS_INFO("Loading XML : %s", xml_filename.c_str());

    // read bt xml
    auto tree = factory.createTreeFromText(xml_filename);

    // Create a logger
    StdCoutLogger logger_cout(tree);

    //Get moveit group name
    robot_obj.GetBasicInfo();
    robot_obj.InitialiseMoveit(nh);

    //bt nodes
    NodeStatus status = NodeStatus::RUNNING;

    while (ros::ok() && status == NodeStatus::RUNNING) {
     status = tree.root_node->executeTick();
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;

}