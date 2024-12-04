#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>  
#include <std_msgs/Bool.h>  
#include <chrono> // 引入 chrono 库，用于时间操作
#include "kuavo_behavior_tree_scene/navigation/robotNavData.hpp"
namespace RobotNav
{
    /**
     * 模拟一个导航功能
    */
    class NavigationToTarget : public BT::StatefulActionNode
    {
    public:
      NavigationToTarget(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
      { 
        // ros句柄
        ros::NodeHandle nh;

        // 数据赋值
        voice_wake_up_ = false;

        // 订阅语音唤醒话题
        voice_wake_up_sub_ = nh.subscribe("/voice_wake_up_status", 10, &NavigationToTarget::voiceWakeUpCallback, this);
      }

      // 定义端口
      static BT::PortsList providedPorts()
      {
        return {
                BT::InputPort<int>("current_nav_point_id"), 
                BT::InputPort<Position3D>("nav_point_pos"), 
                BT::InputPort<Orientation4D>("nav_point_quat"),
                BT::OutputPort<int>("finsh_flag")
        };
      }

        // 开始时调用
        BT::NodeStatus onStart() override
        { 
            start_time_ = std::chrono::steady_clock::now(); // 记录当前时间
            // std::cout << "NavigationToTarget onStart: " << this->name() << std::endl;
            std::cout << " 开始发送导航任务 | 检索导航任务是否为中断状态 " << std::endl;

            // 从端口获取数据并保存到类中
            if (getInput<int>("current_nav_point_id", current_nav_point_id_))
            {
                std::cout << "当前导航点 ID: " << current_nav_point_id_ << std::endl;
            }
            if (getInput<Position3D>("nav_point_pos", nav_point_pos_))
            {
                std::cout << "目标位置(xyz): (" << nav_point_pos_.x << ", " << nav_point_pos_.y << ", " << nav_point_pos_.z << ")" << std::endl;
            }
            if (getInput<Orientation4D>("nav_point_quat", nav_point_quat_))
            {
                std::cout << "目标四元数(wxyz): (" << nav_point_quat_.w << ", " << nav_point_quat_.x << ", " << nav_point_quat_.y << ", " << nav_point_quat_.z << ")" << std::endl;
            }

            return BT::NodeStatus::RUNNING;

            /**
             * （1）使能轨迹转发
             * （2）机器人开始导航行走
            */
        }

        // 每次调用时检查是否超过10秒
        BT::NodeStatus onRunning() override
        { 
            // 检查语音唤醒状态，如果为true，调用halt()中止任务
            if (voice_wake_up_)
            {
                std::cout << "语音唤醒触发，任务中止！" << std::endl;
                halt();  // 调用halt()来中止任务
                
                /** 
                 * （1） 关闭轨迹转发
                 * （2） 机器人停在原地
                */
                
                // 节点返回success提前进入语音唤醒功能
                return BT::NodeStatus::SUCCESS;
            }

            // 记录当前时间
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = now - start_time_;

            // 如果未超过5秒，继续返回 RUNNING
            if (elapsed_seconds.count() < 5.0)
            {
                // 假设导航中
                std::cout << " -------- 导航中 ------------ " << std::endl;

                // 监听到语音触发

                return BT::NodeStatus::RUNNING;
            }

            // 超过5秒后返回 SUCCESS
            std::cout << " -------- 导航完毕 ------------ " << std::endl;
            std::cout << "MoveBaseAction SUCCESS: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        // 被中止时的回调
        void onHalted() override
        { 
            // 触发中断之后，tick应该跳到语音交互的树当中
            std::cout << " -------- 导航中断触发任务 ------------ " << std::endl;
        }

        // 语音唤醒回调函数
        void voiceWakeUpCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            voice_wake_up_ = msg->data;  // 更新语音唤醒状态
        }

    private:
        std::chrono::steady_clock::time_point start_time_; // 用于记录开始时间
        int current_nav_point_id_;  // 存储当前导航点 ID
        Position3D nav_point_pos_;  // 存储目标位置
        Orientation4D nav_point_quat_;  // 存储目标四元数
        bool voice_wake_up_;  // 用于存储语音唤醒状态
        ros::Subscriber voice_wake_up_sub_;  // 语音唤醒话题订阅者
    };
} // namespace RobotNav