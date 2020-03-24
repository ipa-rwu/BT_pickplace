#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_json_dynamic.h"

#include <unistd.h>
#include <stdio.h>

#include "ros/ros.h"
#include "robot_function/robot_function.h"

#include "robot_function/btnodes_param.h"
#include "robot_function/bt.h"
#include "robot_function/btnodes_arm.h"
#include "robot_function/btnodes_gripper.h"
#include "robot_function/btnodes_condition.h"
#include "robot_function/btnodes_others.h"



#include "robot_function/robot_gripper.h"
#include "robot_function/environment.h"

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif


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
    BehaviorTreeFactory factory;

    using namespace BTNodesArm;
    using namespace BTNodesGripper;
    using namespace BTNodesCondition;
    using namespace BTNodesParameter;
    using namespace BTNodesOthers;


    // boost::shared_ptr<Robot_Function> robot_obj;
    ros::init(argc, argv, "kogrob_demo");
    ros::NodeHandle nh;

    // robot function
    RobotFunction robot_obj(nh);
    GripperFunction gripper_obj;
    EnvironmentSet envrionment_obj;
    ParamClient param_obj();

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string xml_filename;
    nh.param<std::string>("file", xml_filename, "/home/rachel/kogrob/kogrob_ws/src/BT_pickplace/robot_function/treexml/PickPlace_sim.xml");
    ROS_INFO("Loading XML : %s", xml_filename.c_str());
    

    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::MoveGroupInterface *gripper_group;

    const std::string GROUP_MANIP = "manipulator";
    const std::string GROUP_GRIPP = "gripper";

    move_group = new moveit::planning_interface::MoveGroupInterface(GROUP_MANIP);
    gripper_group = new moveit::planning_interface::MoveGroupInterface(GROUP_GRIPP);

    
    //robot_obj.reset(new Robot_Function);
    robot_obj.InitialiseMoveit(nh, move_group);
    robot_obj.GetBasicInfo(move_group);
    move_group->setMaxVelocityScalingFactor(0.5);

    gripper_obj.InitialiseGripper(nh, gripper_group);

    envrionment_obj.AddCollissionObjects(nh, move_group);

    ROS_INFO("---------------------------");
    move_group->setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group->plan(my_plan);
    move_group->move();
    //Get moveit group name
    // robot_obj.InitialiseMoveit(nh);
    // robot_obj.GetBasicInfo();

    // using namespace RobotControl;

    // We use the BehaviorTreeFactory to register our custom nodes

    // Note: the name used to register should be the same used in the XML.
    // using namespace BehaviorTreeNodes;

    // print current path
    char cwd[1024];
    chdir("/path/to/change/directory/to");
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);

    // wait for target
    NodeBuilder builder_pathplan = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<APathPlanning>( name, config, nh, move_group);
    };
    factory.registerBuilder<APathPlanning>( "APathPlanning",builder_pathplan);

    NodeBuilder builder_executeplan = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        return std::make_unique<AFollowPath>( name, config, nh, move_group);
    };
    factory.registerBuilder<AFollowPath>( "AFollowPath",builder_executeplan);

    NodeBuilder builder_grippermove = [&nh, &gripper_group](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<AGripperMove>( name, config, nh, gripper_group);
    };
    factory.registerBuilder<AGripperMove>( "AGripperMove", builder_grippermove);


    // schunk
    NodeBuilder builder_schunkmove = [&nh, &gripper_group](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<AGripperMoveSchunk>( name, config, nh, gripper_group);
    };
    factory.registerBuilder<AGripperMoveSchunk>( "AGripperMoveSchunk", builder_schunkmove);


    // Camera find target
    NodeBuilder builder_camerafindtarget = [&nh](const std::string& name, const NodeConfiguration& config)
    {
        std::cout << "Initial CameraFindTarget " << std::endl;
        return std::make_unique<AFindObjContainers>( name, config, nh);
    };
    factory.registerBuilder<AFindObjContainers>( "AFindObjContainers", builder_camerafindtarget);

    NodeBuilder builder_closetotarget = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<APreparePoseArm>( name, config, nh, move_group);
    };
    factory.registerBuilder<APreparePoseArm>( "APreparePoseArm", builder_closetotarget);

    NodeBuilder builder_checkcondition = [&nh, &move_group](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<ACheckConditionArm>( name, config, nh, move_group);
    };
    factory.registerBuilder<ACheckConditionArm>( "ACheckConditionArm", builder_checkcondition);

    NodeBuilder builder_reloadparam = [&nh](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<AReloadParam>( name, config, nh, true, false);
    };
    factory.registerBuilder<AReloadParam>( "AReloadParam", builder_reloadparam);

    NodeBuilder builder_setmarker = [](const std::string& name, const NodeConfiguration& config)
    {
        
        return std::make_unique<ASetMarker>( name, config, 1);
    };
    factory.registerBuilder<ASetMarker>( "ASetMarker", builder_setmarker);


    factory.registerNodeType<ACheckConditionFlag>("ACheckConditionFlag");

    factory.registerNodeType<ASetFlag>("ASetFlag");

    factory.registerNodeType<APrepareGripper>("APrepareGripper");

    factory.registerNodeType<ACheckConditionLoad>("ACheckConditionLoad");
    auto tree = factory.createTreeFromFile(xml_filename);

    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    // This logger saves state changes on file
    // FileLogger logger_file(tree, "bt_trace.fbl");

    // This logger stores the execution time of each node
    // MinitraceLogger logger_minitrace(tree, "bt_trace.json");
    JsonDynamicLogger logger_json_file(tree, "bt_trace.json");
#ifdef ZMQ_FOUND
    PublisherZMQ publisher_zmq(tree);
#endif

    printTreeRecursively(tree.root_node);
    NodeStatus status = NodeStatus::RUNNING;

    while (ros::ok()) {
     status = tree.root_node->executeTick();
        // Sleep 100 milliseconds
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
