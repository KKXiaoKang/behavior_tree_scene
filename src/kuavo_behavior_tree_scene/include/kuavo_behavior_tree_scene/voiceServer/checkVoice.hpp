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
    class CheckVoice : public BT::ConditionNode
    {
    public:
        CheckVoice(const std::string& name, const BT::NodeConfiguration& config)
            : BT::ConditionNode(name, config), voice_wake_up_status_(false)
        {
            ros::NodeHandle nh;
            voice_wake_up_sub_ = nh.subscribe("/voice_wake_up_status", 10, &CheckVoice::voiceWakeUpCallback, this);
        }

        static BT::PortsList providedPorts()
        {
            return {
              //   BT::InputPort<int>("battery_level"),  // 输入端口：电池电量
              //   BT::OutputPort<std::string>("battery_status")  // 输出端口：电池状态
            };
        }

        BT::NodeStatus tick() override
        {
            std::cout << "CheckVoice: 检测语音唤醒状态..." << std::endl;

            // 如果语音唤醒状态为 true，返回 SUCCESS；否则返回 FAILURE
            if (voice_wake_up_status_)
            {
                std::cout << "语音唤醒状态: SUCCESS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "语音唤醒状态: FAILURE" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }

    private:
        ros::Subscriber voice_wake_up_sub_;  // 订阅器
        bool voice_wake_up_status_;         // 语音唤醒状态

        void voiceWakeUpCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            voice_wake_up_status_ = msg->data;
            // std::cout << "接收到语音唤醒状态: " << (voice_wake_up_status_ ? "TRUE" : "FALSE") << std::endl;
        }
    };
} // namespace VoiceServer