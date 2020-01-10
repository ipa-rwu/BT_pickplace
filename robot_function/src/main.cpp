#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include <unistd.h>
#include <stdio.h>

#include "ros/ros.h"
#include "robot_function.h"
#include "bt_nodes.h"

using namespace BT;

static const char* xml_text = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <OpenGripper name="open_gripper"/>
            <CloseGripper name="close_gripper"/>
        </Sequence>
 </root>
 )";

 int main(int argc, char **argv)
{
    // boost::shared_ptr<Robot_Function> robot_obj;
    ros::init(argc, argv, "kogrob_demo");
    ros::NodeHandle nh;

    // robot function
    RobotFunction robot_obj;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string xml_filename;
    nh.param<std::string>("file", xml_filename, "/home/rachel/kogrob/kogrob_ws/src/robot_function/treexml/test_tree.xml");
    ROS_INFO("Loading XML : %s", xml_filename.c_str());
    
    // Robot_Function robot_obj;
    //robot_obj.reset(new Robot_Function);
    //robot_obj->initialiseMoveit(nh);
    //robot_obj->printBasicInfo();
    ROS_INFO("---------------------------");

        //Get moveit group name
    robot_obj.InitialiseMoveit(nh);
    robot_obj.GetBasicInfo();

    using namespace RobotControl;

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // Note: the name used to register should be the same used in the XML.
    // using namespace BehaviorTreeNodes;

    // print current path
    char cwd[1024];
    chdir("/path/to/change/directory/to");
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);

   
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));

    NodeBuilder builder_A = [&robot_obj](const std::string& name, const NodeConfiguration& config)
    {

        geometry_msgs::Pose target_pose;
        target_pose.position.x = 0.4;
        target_pose.position.y = 0.4;
        target_pose.position.z = 0.012;
        target_pose.orientation.w=1.0;
        pathplan rob_pathplan = robot_obj.PathPlanning(target_pose);
        bool success = rob_pathplan.success;
        moveit::planning_interface::MoveGroupInterface::Plan plan=rob_pathplan.plan;
        return std::make_unique<BTPathPlanning>( name, config, success, plan);
    };

    // BehaviorTreeFactory::registerBuilder is the more general way to register a custom node.
    // Not the most user friendly, but definitely the most flexible one.
    factory.registerBuilder<BTPathPlanning>( "BTPathPlanning", builder_A);


    NodeBuilder builder_B = [&robot_obj](const std::string& name, const NodeConfiguration& config)
    {
        BTFollowPath::tick bt;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        //PortsList planedpath = { InputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan")};
        auto res = bt.getInput("pathplan", plan);
        if( !res )
        {
        throw RuntimeError("error reading port [planedpath]:", res.error() );
        }
        bool success = robot_obj.MoveGroupExecutePlan(plan);
        return std::make_unique<BTFollowPath>( name, config, success);
    };
    factory.registerBuilder<BTFollowPath>( "BTFollowPath", builder_B);

    //PortsList robot_object_ports = { InputPort<boost::shared_ptr<Robot_Function>>(robot_obj) };
    //factory.registerSimpleAction("ApproachObject", ApproachObject, robot_object_ports );

/*
    // SimpleActionNodes can not define their own method providedPorts(), therefore
    // we have to pass the PortsList explicitly if we want the Action to use getInput()
    // or setOutput();
    PortsList say_something_ports = { InputPort<std::string>("message") };
    factory.registerSimpleAction("SaySomething2", SaySomethingSimple, say_something_ports );
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");
*/
    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    // auto tree = factory.createTreeFromFile("src/bt_sample/tree_xml/test_tree.xml");
    auto tree = factory.createTreeFromFile(xml_filename);

    // Create a logger
    StdCoutLogger logger_cout(tree);

    // Create some loggers.
//    StdCoutLogger   logger_cout(tree);
//    FileLogger      logger_file(tree, "bt_trace.fbl");
//    MinitraceLogger logger_minitrace(tree, "bt_trace.json");
    NodeStatus status = NodeStatus::RUNNING;

    while (ros::ok() && status == NodeStatus::RUNNING) {
     status = tree.root_node->executeTick();
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    // tree.root_node->executeTick();

    return 0;
}


// int main(int argc, char **argv)
// {
//     /*
//     // print current path
//     char cwd[1024];
//     chdir("/path/to/change/directory/to");
//     getcwd(cwd, sizeof(cwd));
//     printf("Current working dir: %s\n", cwd);
//     */

//     ros::init(argc, argv, "kogrob_demo");
//     ros::NodeHandle nh;
    
//     // robot function
//     RobotFunction robot_obj;

//     // behavior tree nodes
//     // using namespace RobotControl;
//     BehaviorTreeFactory factory;

//     // ros::AsyncSpinner spinner(1);
//     // spinner.start();
    
//     std::string xml_filename;
//     nh.param<std::string>("file", xml_filename, "/home/rachel/kogrob/kogrob_ws/src/robot_function/treexml/test_tree.xml");
//     ROS_INFO("Loading XML : %s", xml_filename.c_str());

//     // //Get moveit group name
//     // robot_obj.GetBasicInfo();
//     // robot_obj.InitialiseMoveit(nh);


//     // 
//     // NodeBuilder builder_A = [&robot_obj](const std::string& name, const NodeConfiguration& config)
//     // {

//     //     geometry_msgs::Pose target_pose;
//     //     target_pose.position.x = 0.1;
//     //     target_pose.position.y = 0.2;
//     //     target_pose.position.z = 0.3;
//     //     target_pose.orientation.w=1.0;
//     //     pathplan rob_pathplan = robot_obj.PathPlanning(target_pose);
//     //     bool success = rob_pathplan.success;
//     //     return std::make_unique<BTPathPlanning>( name, config, success);
//     // };

//     // // BehaviorTreeFactory::registerBuilder is the more general way to register a custom node.
//     // // Not the most user friendly, but definitely the most flexible one.
//     // factory.registerBuilder<BTPathPlanning>( "BTPathPlanning", builder_A);

//     GripperInterface gripper;
//     factory.registerSimpleAction("OpenGripper", 
//                                  std::bind(&GripperInterface::open, &gripper));
//     factory.registerSimpleAction("CloseGripper", 
//                                  std::bind(&GripperInterface::close, &gripper));

//         // read bt xml
//     auto tree = factory.createTreeFromText(xml_text);

//     // Create a logger
//     StdCoutLogger logger_cout(tree);

//     //bt nodes
//     NodeStatus status = NodeStatus::RUNNING;

//     while (ros::ok() && status == NodeStatus::RUNNING) {
//      status = tree.root_node->executeTick();
//         // Sleep 100 milliseconds
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }

//     return 0;

// }