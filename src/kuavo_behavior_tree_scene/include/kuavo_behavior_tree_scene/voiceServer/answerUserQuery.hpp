#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>  
#include <std_msgs/Bool.h>  
#include <chrono> // 引入 chrono 库，用于时间操作
#include "kuavo_behavior_tree_scene/navigation/robotNavData.hpp"

namespace VoiceServer
{
    /**
     * 模拟一个语音唤醒功能
    */
    class AnswerUserQuery : public BT::SyncActionNode
    {
    public:
      AnswerUserQuery(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
      { }

      static BT::PortsList providedPorts()
      {
          return {
              BT::OutputPort<int>("real_point_id"),
          };
      }

      BT::NodeStatus tick() override
      {
        std::cout << " ---------------- 回答用户问题当中: -------------------- " << this->name() << std::endl;

        /**
         * （1）在这里持续不断的进行语音问答 | 先询问客户需要什么帮助 | 是协助还是护送导航？
         * （2）等到用户说明协助完毕之后，进入到导航状态
         * 如果是协助那就正常语音对话，对话完毕之后就 return BT::NodeStatus::SUCCESS
         * 如果是护送导航，那就这个节点 return BT::NodeStatus::FAILURE;
        */
        ros::Duration(5.0).sleep();  // 等待5秒

        // 设置导航点为5号位置
        int nav_point_id = 5;
        setOutput("real_point_id", nav_point_id);

        // return BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::FAILURE;
      }
    };
} // namespace VoiceServer