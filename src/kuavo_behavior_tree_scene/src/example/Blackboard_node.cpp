#include <ros/ros.h>
#include <ros/package.h> 

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>


// CheckBattery 节点
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    BT::Optional<std::string> msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error() );
    }
    std::cout << "CheckBattery: " << this->name() << std::endl;
    std::cout << "CheckBattery says: " << msg.value() << std::endl;

    ros::Duration(5.0).sleep();  // 等待5秒
    return BT::NodeStatus::SUCCESS;
  }
};

class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("text") };
  }

  BT::NodeStatus tick() override
  {
    std::cout << "OpenGripper: " << this->name() << std::endl;
    setOutput<std::string>("text", "The answer is 42");
    ros::Duration(5.0).sleep();  // 等待5秒
    return BT::NodeStatus::SUCCESS;
  }
};

class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    BT::Optional<std::string> msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error() );
    }
    std::cout << "ApproachObject: " << this->name() << std::endl;
    std::cout << "ApproachObject says: " << msg.value() << std::endl;

    ros::Duration(5.0).sleep();  // 等待5秒
    return BT::NodeStatus::SUCCESS;
  }
};

class CloseGripper : public BT::SyncActionNode
{
public:
  CloseGripper(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("new_message") };
  }

  BT::NodeStatus tick() override
  {
    BT::Optional<std::string> msg = getInput<std::string>("new_message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [new_message]: ", 
                              msg.error() );
    }
    std::cout << "CloseGripper: " << this->name() << std::endl;
    std::cout << "CloseGripper says: " << msg.value() << std::endl;
    ros::Duration(5.0).sleep();  // 等待5秒
    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_example_node");
    ros::NodeHandle nh;
    std::string package_path = ros::package::getPath("kuavo_behavior_tree_scene");
    std::string tree_file_path = package_path + "/config/example/Blackboard_tree.xml";

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CheckBattery>("check_battery");
    factory.registerNodeType<OpenGripper>("open_gripper");
    factory.registerNodeType<ApproachObject>("approach_object");
    factory.registerNodeType<CloseGripper>("close_gripper");

    ROS_INFO("All nodes registered.");

    auto tree = factory.createTreeFromFile(tree_file_path);

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        BT::NodeStatus status = tree.tickRoot();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
