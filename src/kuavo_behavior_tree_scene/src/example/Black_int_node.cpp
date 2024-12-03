#include <ros/ros.h>
#include <ros/package.h> 
#include <chrono> // 引入 chrono 库，用于时间操作

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// We want to use this custom type
struct Position2D 
{ 
  double x;
  double y; 
};

// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline Position2D convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Position2D output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            return output;
        }
    }
} // end namespace BT

// CheckBattery 节点
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    // return { BT::OutputPort<std::string>("message") };
    return { BT::OutputPort<Position2D>("message") };
  }

  BT::NodeStatus tick() override
  {
    std::cout << "CheckBattery: " << this->name() << std::endl;
    Position2D mygoal = {1.1, 2.3};
    setOutput<Position2D>("message", mygoal);
    ros::Duration(2.0).sleep();  // 等待5秒
    return BT::NodeStatus::SUCCESS;
  }
};


// OpenGripper 节点
class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    //return { BT::InputPort<std::string>("text") };
    const char*  description = "Simply print the goal on console...";
    return { BT::InputPort<Position2D>("text", description) };
  }

  BT::NodeStatus tick() override
  {
    //BT::Optional<std::string> msg = getInput<std::string>("text");
    auto res = getInput<Position2D>("text");
    // if (!msg)
    // {
    //   throw BT::RuntimeError("missing required input [text]: ", 
    //                           msg.error() );
    // }
    // std::cout << "OpenGripper: " << this->name() << std::endl;
    // std::cout << "OpenGripper says: " << msg.value() << std::endl;

    // ros::Duration(2.0).sleep();  // 等待5秒
    // return BT::NodeStatus::SUCCESS;
    if( !res )
    {
      throw BT::RuntimeError("error reading port [text]:", res.error());
    }
    Position2D target = res.value();
    printf("OpenGripper Target positions: [ %.1f, %.1f ]\n", target.x, target.y );
    
    return BT::NodeStatus::SUCCESS;
    // return BT::NodeStatus::FAILURE;
  }
};

// --------------------------------------------------------------------------- // 

// ApproachObject
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    //return { BT::OutputPort<std::string>("new_message") };
    return { BT::OutputPort<Position2D>("new_message") };
  }

  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    // setOutput<std::string>("new_message", "hhh --- hkkk");
    // ros::Duration(2.0).sleep();  // 等待5秒
    // return BT::NodeStatus::SUCCESS;
    Position2D mygoal = {9.9, 2.111};
    setOutput<Position2D>("new_message", mygoal);
    ros::Duration(2.0).sleep();  // 等待5秒
    
    return BT::NodeStatus::SUCCESS;
    // return BT::NodeStatus::FAILURE;
  }
};

// CloseGripper
class CloseGripper : public BT::SyncActionNode
{
public:
  CloseGripper(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    // return { BT::InputPort<std::string>("new_new_message") };
    const char*  description = "Simply print the goal on console...";
    return { BT::InputPort<Position2D>("new_new_message", description) };
  }

  BT::NodeStatus tick() override
  {
    // BT::Optional<std::string> msg = getInput<std::string>("new_new_message");
    // if (!msg)
    // {
    //   throw BT::RuntimeError("missing required input [new_new_message]: ", 
    //                           msg.error() );
    // }
    // std::cout << "CloseGripper: " << this->name() << std::endl;
    // std::cout << "CloseGripper says: " << msg.value() << std::endl;
    // ros::Duration(2.0).sleep();  // 等待5秒
    // return BT::NodeStatus::SUCCESS;
    auto res = getInput<Position2D>("new_new_message");
    if( !res )
    {
      throw BT::RuntimeError("error reading port [new_new_message]:", res.error());
    }
    Position2D target = res.value();
    printf("CloseGripper positions: [ %.1f, %.1f ]\n", target.x, target.y );
    return BT::NodeStatus::SUCCESS;
  }
};

// MoveBaseAction 
class MoveBaseAction : public BT::StatefulActionNode
{
public:
    // 构造函数
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config),
          start_time_(std::chrono::steady_clock::now()) // 初始化开始时间
    {}

    // 定义端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Simply print the goal on console...";
        return { BT::InputPort<Position2D>("set_message", description) };
    }

    // 开始时调用
    BT::NodeStatus onStart() override
    { 
        start_time_ = std::chrono::steady_clock::now(); // 记录当前时间
        std::cout << "MoveBaseAction onStart: " << this->name() << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    // 每次调用时检查是否超过10秒
    BT::NodeStatus onRunning() override
    { 
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = now - start_time_;
        
        // 如果未超过2秒，继续返回 RUNNING
        if (elapsed_seconds.count() < 2.0)
        {
            std::cout << "MoveBaseAction onRunning: " << this->name() << std::endl;
            return BT::NodeStatus::RUNNING;
        }

        // 超过10秒后返回 SUCCESS
        std::cout << "MoveBaseAction SUCCESS: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    // 被中止时的回调
    void onHalted() override
    { 
        printf("[ MoveBase: ABORTED ]");
    }

private:
    std::chrono::steady_clock::time_point start_time_; // 用于记录开始时间
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_example_node");
    ros::NodeHandle nh;
    std::string package_path = ros::package::getPath("kuavo_behavior_tree_scene");
    std::string tree_file_path = package_path + "/config/example/Black_int.xml";

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CheckBattery>("check_battery");
    factory.registerNodeType<OpenGripper>("open_gripper");
    factory.registerNodeType<ApproachObject>("approach_object");
    factory.registerNodeType<CloseGripper>("close_gripper");
    factory.registerNodeType<MoveBaseAction>("move_base_action");

    ROS_INFO("All nodes registered.");

    auto tree = factory.createTreeFromFile(tree_file_path);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        BT::NodeStatus status = tree.tickRoot();
        std::cout << "\n--- ticking\n";
        std::cout << "--- status: " << toStr(status) << "\n";
        std::cout << "--- ticking\n\n";
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
