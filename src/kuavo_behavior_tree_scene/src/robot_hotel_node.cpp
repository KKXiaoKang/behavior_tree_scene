#include <ros/ros.h>
#include <ros/package.h> 
#include <chrono> // 引入 chrono 库，用于时间操作

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// navigation
#include "kuavo_behavior_tree_scene/navigation/navigationToTarget.hpp"
#include "kuavo_behavior_tree_scene/navigation/taskIdOK.hpp"

// voice server
#include "kuavo_behavior_tree_scene/voiceServer/checkVoice.hpp"
#include "kuavo_behavior_tree_scene/voiceServer/answerUserQuery.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_example_node");
    ros::NodeHandle nh;
    std::string package_path = ros::package::getPath("kuavo_behavior_tree_scene");
    std::string tree_file_path = package_path + "/config/navigation/navigation_tree.xml";

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<RobotNav::TaskIdOK>("TaskIdOK");
    factory.registerNodeType<RobotNav::NavigationToTarget>("NavigationToTarget");

    factory.registerNodeType<VoiceServer::CheckVoice>("CheckVoice");
    factory.registerNodeType<VoiceServer::AnswerUserQuery>("AnswerUserQuery");
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
