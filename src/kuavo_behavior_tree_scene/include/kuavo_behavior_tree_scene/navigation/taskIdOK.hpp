#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>  // 添加这一行
#include "kuavo_behavior_tree_scene/navigation/robotNavData.hpp"

namespace RobotNav
{
    /**
     * @brief This class is a condition node that checks if the task id is equal to the expected task id.
     * 任务执行状态
       名称：/state_motion_control
       格式：std_msgs/String
       
       状态：
       	IDEL    -- 空闲
       	ARRIVED -- 到达
       	FAILURE -- 失败
       
       其他状态：
       	FOLLOW -- 任务执行中，且距离目标点较远
       	ARRIVING -- 任务执行中，即将到达目标点，减速
       	ADJUST_DIRECTION -- 任务执行中，到达目标点位置，调整机器人朝向
     * @param name Name of the node
     * @param config Node configuration
    */
    class TaskIdOK : public BT::ConditionNode
    {
    public:
      TaskIdOK(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
      { 
        ros::NodeHandle nh;
        // 数据初始化
        current_state_ = "FOLLOW";
        
        // 导航状态订阅
        nav_state_sub_ = nh.subscribe("/state_motion_control", 10, &TaskIdOK::stateCallback, this);
      }

      // 定义端口
      static BT::PortsList providedPorts()
      {
        return {
                BT::InputPort<int>("nav_point_id"), 
                BT::OutputPort<Position3D>("nav_point_pos"), 
                BT::OutputPort<Orientation4D>("nav_point_quat"),
                BT::OutputPort<int>("current_nav_point_id"),
        };
      }

      /** 
       * @brief 检查是否满足条件可以进行导航
      */
      BT::NodeStatus tick() override
      {
        // 设置端口信息数据
        int nav_point_id = 0;
        
        Position3D nav_point_pos;
        nav_point_pos.x = 0.0;
        nav_point_pos.y = 0.0;
        nav_point_pos.z = 0.0;

        Orientation4D nav_point_quat;
        nav_point_quat.x = 0.0;
        nav_point_quat.y = 0.0;
        nav_point_quat.z = 0.0;
        nav_point_quat.w = 1.0;
        
        // 获取输入端口数据
        getInput("nav_point_id", nav_point_id);
        int current_nav_point_id = nav_point_id;

        // 设置输出端口数据
        setOutput("nav_point_pos", nav_point_pos);
        setOutput("nav_point_quat", nav_point_quat);
        setOutput("current_nav_point_id", current_nav_point_id);
        std::cout << "TaskIdOK: current_nav_point_id = " << current_nav_point_id << std::endl;
        // 等待2s
        ros::Duration(2.0).sleep();

        // 根据当前状态判断任务是否完成
        if (current_state_ == "ARRIVED" || current_state_ == "FOLLOW" || current_state_ == "IDEL")
        {
          return BT::NodeStatus::SUCCESS;
        }
        else if (current_state_ == "FAILURE")
        {
          return BT::NodeStatus::FAILURE;
        }
        else
        {
          return BT::NodeStatus::RUNNING;
        }
      }
    
    private:
      // 回调函数，用于更新任务状态
      void stateCallback(const std_msgs::String::ConstPtr& msg)
      {
        current_state_ = msg->data;  // 保存当前状态
      }

      ros::Subscriber nav_state_sub_;
      std::string current_state_;  // 当前任务状态
    };
} // namespace RobotNav