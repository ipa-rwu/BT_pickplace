#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include <unistd.h>
#include <stdio.h>

#include "ros/ros.h"
#include "robot_function/robot_function.h"
// #include "bt_nodes.h"
#include "robot_function/bt.h"


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
    RobotFunction robot_obj(nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string xml_filename;
    nh.param<std::string>("file", xml_filename, "/home/rachel/kogrob/kogrob_ws/src/robot_function/treexml/tree4.xml");
    ROS_INFO("Loading XML : %s", xml_filename.c_str());
    

    moveit::planning_interface::MoveGroupInterface *move_group;
    const std::string GROUP_MANIP = "manipulator";
    move_group = new moveit::planning_interface::MoveGroupInterface(GROUP_MANIP);
    
    const std::string GROUP_GRIPP = "endeffector";
    //robot_obj.reset(new Robot_Function);
    // robot_obj.InitialiseMoveit(nh, *move_group);
    robot_obj.GetBasicInfo(move_group);
    ROS_INFO("---------------------------");

    //Get moveit group name
    // robot_obj.InitialiseMoveit(nh);
    // robot_obj.GetBasicInfo();

    // using namespace RobotControl;

    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // Note: the name used to register should be the same used in the XML.
    // using namespace BehaviorTreeNodes;

    // print current path
    char cwd[1024];
    chdir("/path/to/change/directory/to");
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);

   
    // GripperInterface gripper;
    // factory.registerSimpleAction("OpenGripper", 
    //                              std::bind(&GripperInterface::open, &gripper));
    // factory.registerSimpleAction("CloseGripper", 
    //                              std::bind(&GripperInterface::close, &gripper));

    // pathplan global_plan;
    // geometry_msgs::Pose target_pose;


    // NodeBuilder builder_pathplanning = [&robot_obj, &global_plan, &target_pose](const std::string& name, const NodeConfiguration& config)
    // {
    //       std::cout<<"target_pose.position.x: "<< target_pose.position.x << std::endl;

    //     global_plan = robot_obj.PathPlanning(target_pose);
    //     // pathplan rob_pathplan = robot_obj.PathPlanning(target_pose);
    //     bool success = global_plan.success;
    //     moveit::planning_interface::MoveGroupInterface::Plan plan=global_plan.plan;
    //     // bool success = rob_pathplan.success;
    //     // moveit::planning_interface::MoveGroupInterface::Plan plan=rob_pathplan.plan;
    //     return std::make_unique<BTPathPlanning>( name, config, success, plan);
    // };
    // factory.registerBuilder<BTPathPlanning>( "BTPathPlanning", builder_pathplanning);
    // PortsList planedpath = { InputPort<moveit::planning_interface::MoveGroupInterface::Plan>("pathplan")};

    // NodeBuilder builder_move = [&robot_obj, &global_plan](const std::string& name, const NodeConfiguration& config)
    // {
    //     moveit::planning_interface::MoveGroupInterface::Plan plan = global_plan.plan;
    //     bool success = robot_obj.MoveGroupExecutePlan(plan);
    //     return std::make_unique<BTFollowPath>( name, config, success);
    // };
    // factory.registerBuilder<BTFollowPath>( "BTFollowPath", builder_move);

    // NodeBuilder builder_waitfortarget = [](const std::string& name, const NodeConfiguration& config)
    // {
    //     PortsList waitfortarget = { OutputPort<geometry_msgs::Pose>("target") };
    //     return std::make_unique<BTWaitForTarget>( name, config);
    // };

    // wait for target
    factory.registerNodeType<BTWaitForTarget>( "BTWaitForTarget");

    NodeBuilder builder_pathplan = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<BTPathPlanning>( name, config, nh, move_group);
    };
    factory.registerBuilder<BTPathPlanning>( "BTPathPlanning",builder_pathplan);

    NodeBuilder builder_executeplan = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<BTFollowPath>( name, config, nh, move_group);
    };
    factory.registerBuilder<BTFollowPath>( "BTFollowPath",builder_executeplan);

    // Camera find target
    NodeBuilder builder_camerafindtarget = [&nh](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<BTCameraFindTarget>( name, config, nh);
    };
    factory.registerBuilder<BTCameraFindTarget>( "BTCameraFindTarget", builder_camerafindtarget);

    NodeBuilder builder_closetotarget = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<BTCloseToTarget>( name, config, nh, move_group);
    };
    factory.registerBuilder<BTCloseToTarget>( "BTCloseToTarget", builder_closetotarget);

    NodeBuilder builder_checkconfition = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<BTCheckCondition>( name, config, nh, move_group);
    };
    factory.registerBuilder<BTCheckCondition>( "BTCheckCondition", builder_checkconfition);
    

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

    while (ros::ok()) {
     status = tree.root_node->executeTick();
        // Sleep 100 milliseconds
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

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