#include <ros/ros.h>
#include <ros/package.h> 

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>


/**
 * BT::NodeStatus::SUCCESS
    *  当一个动作节点（如 SyncActionNode）成功执行了它的操作时，返回 SUCCESS 表示该任务已完成且不再需要继续执行
 * BT::NodeStatus::FAILURE
    *  当一个节点的行为失败，不能达到预期的目标时，返回 FAILURE。例如，动作节点可能遇到错误或失败的条件\
 * BT::NodeStatus::RUNNING
    *  当节点仍在执行过程中，不能立即得出结果时，例如需要等待某个条件满足或者执行一个长时间的操作（如等待 5 秒）。如果一个节点没有完成它的任务并且需要持续运行，就返回 RUNNING
 * BT::NodeStatus::IDLE
    *  用于标示节点当前处于空闲或无操作状态
 * BT::NodeStatus::ERROR
    *  当节点遇到严重错误或者系统异常，无法继续运行时，可以返回 ERROR。虽然 ERROR 并不是 NodeStatus 的标准状态之一，但可以通过自定义行为树节点时返回这种状态
*/


// CheckBattery 节点
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "CheckBattery: " << this->name() << std::endl;

    // 模拟等待5秒钟
    ros::Duration(5.0).sleep();  // 等待5秒

    return BT::NodeStatus::SUCCESS;
  }
};
class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "OpenGripper: " << this->name() << std::endl;

    // 模拟等待5秒钟
    ros::Duration(5.0).sleep();  // 等待5秒

    // return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::SUCCESS;
  }
};
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;

    // 模拟等待5秒钟
    ros::Duration(5.0).sleep();  // 等待5秒

    return BT::NodeStatus::SUCCESS;
  }
};
class CloseGripper : public BT::SyncActionNode
{
public:
  CloseGripper(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    std::cout << "CloseGripper: " << this->name() << std::endl;

    // 模拟等待5秒钟
    ros::Duration(5.0).sleep();  // 等待5秒

    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_example_node");
    ros::NodeHandle nh;
    std::string package_path = ros::package::getPath("kuavo_behavior_tree_scene");
    std::string tree_file_path = package_path + "/config/example/my_tree.xml";

    // 使用 BehaviorTreeFactory 注册自定义节点
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CheckBattery>("check_battery");
    factory.registerNodeType<OpenGripper>("open_gripper");
    factory.registerNodeType<ApproachObject>("approach_object");
    factory.registerNodeType<CloseGripper>("close_gripper");

    ROS_INFO("All nodes registered.");
    // BT::Groot2Publisher publisher(tree);

    // 从文件加载行为树
    auto tree = factory.createTreeFromFile(tree_file_path);

    ros::Rate loop_rate(20);  // 设定循环频率为 20Hz
    while (ros::ok())
    {
        // 每次循环时执行行为树
        BT::NodeStatus status = tree.tickRoot();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
