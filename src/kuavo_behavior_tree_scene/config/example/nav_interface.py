#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import math
from dx_nav_common.srv import SetTaskSrv, SetTaskSrvRequest, SetTaskSrvResponse
from dx_nav_common.srv import SetParamsSrv, SetParamsSrvRequest, SetParamsSrvResponse
from nav_msgs.msg import Odometry, MapMetaData
from std_msgs.msg import String, Int32
from dx_nav_common.msg import StateTaskExecute, InitPose, TaskNodes


class RobotNavigationInterface:
    def __init__(self, is_init=False):
        if is_init:
            rospy.init_node('robot_nav_interface', anonymous=True)
        self.add_task = rospy.ServiceProxy("/srv_set_task", SetTaskSrv)
        self.switch_map = rospy.ServiceProxy("/srv_set_params", SetParamsSrv)

    def get_robot_position(self):
        """等待并获取一次机器人位置消息"""
        data = rospy.wait_for_message("/fuse_pose", Odometry)
        ori_data = data.pose.pose
        x = ori_data.position.x
        y = ori_data.position.y
        qx = ori_data.orientation.x
        qy = ori_data.orientation.y
        qz = ori_data.orientation.z
        qw = ori_data.orientation.w

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        theta = math.atan2(siny_cosp, cosy_cosp)

        x = round(x, 3)
        y = round(y, 3)
        theta = round(theta, 3)

        return (x, y, theta)

    def get_robot_left_path(self):
        """获取机器人路径消息"""
        data = rospy.wait_for_message("/state_task_execute", StateTaskExecute)
        return data.left_path

    def get_map_meta_data(self):
        """获取地图信息"""
        data = rospy.wait_for_message("/map_metadata", MapMetaData)

        data_dict = {
            'resolution': data.resolution,
            'origin': {
                'position': {
                    'x': data.origin.position.x,
                    'y': data.origin.position.y,
                    'z': data.origin.position.z
                },
                'orientation': {
                    'x': data.origin.orientation.x,
                    'y': data.origin.orientation.y,
                    'z': data.origin.orientation.z,
                    'w': data.origin.orientation.w
                }
            }
        }
        return data_dict

    def get_current_task_point_id(self):
        """获取机器人任务目标点"""
        data = rospy.wait_for_message("/state_task_execute", StateTaskExecute)
        return data.cur_task.id

    def get_state_derail(self):
        """获取机器人路径状态"""
        data = rospy.wait_for_message("/state_derail", String)
        return data.data

    def get_robot_status(self):
        """获取机器人状态消息"""
        data = rospy.wait_for_message("/state_motion_control", String)
        # rospy.loginfo(f"State Motion Control: {data.data}")
        return data.data

    def robot_switch_map(self, map_dir, vel_max_linear=0.5, vel_max_angular=0.08, method_nav='', method_obstacle_avoid='Stop'):
        """切换地图功能"""
        try:
            request = SetParamsSrvRequest()
            request.map_dir = map_dir
            request.vel_max_linear = vel_max_linear
            request.vel_max_angular = vel_max_angular
            request.method_nav = method_nav
            request.method_obstacle_avoid = method_obstacle_avoid
            response = self.switch_map(request)
            return response.cur_map_dir
        except rospy.ServiceException as e:
            rospy.logerr(f"Service srv_set_params call failed: {e}")
            return None

    def robot_set_pose_station(self, target_point):
        """设置机器人位姿功能"""
        pub = rospy.Publisher('/init_pose', InitPose, queue_size=10)
        rospy.sleep(0.5)
        msg = InitPose()
        msg.station_id = target_point
        msg.method = 'Station'
        pub.publish(msg)
        return True

    def robot_get_task_point_pos(self, target_point):
        data = rospy.wait_for_message("/task_nodes", TaskNodes)
        for item in data.tasks:
            if item.id == target_point:
                return item.pose.x, item.pose.y, item.pose.theta

    def robot_get_task_point_name(self, target_point):
        data = rospy.wait_for_message("/task_nodes", TaskNodes)
        for item in data.tasks:
            if item.id == target_point:
                return item.name

    def robot_get_task_point_list(self):
        data = rospy.wait_for_message("/task_nodes", TaskNodes)
        task_list = []
        for task_info in data.tasks:
            task_dict = {
                'id': task_info.id,
                'name': task_info.name,
                'attr_task': task_info.attr_task,
                'time_delay': task_info.time_delay,
                'pose': {
                    'x': task_info.pose.x,
                    'y': task_info.pose.y,
                    'theta': task_info.pose.theta
                }
            }
            task_list.append(task_dict)
        tasks = json.dumps({'tasks': task_list}, ensure_ascii=False)
        return tasks

    def robot_state_location(self):
        """机器人定位准确判断"""
        data = rospy.wait_for_message("/state_location", Int32)
        # rospy.loginfo(f"State Location Control: {data}")
        return data.data

    def robot_add_task(self, cmd_attr='', task_attr='', task_id=0, task_name=''):
        """
        发布任务点
        cmd_attr: New 为发布任务点, Cancel为取消任务点
        """
        try:
            request = SetTaskSrvRequest()
            request.cmd_attr = cmd_attr
            request.task_attr = task_attr
            request.task_id = task_id
            request.task_name = task_name
            response = self.add_task(request)
            rospy.loginfo(f"Received parameters: cmd_attr={cmd_attr}, task_attr={task_attr}, task_id={task_id}, task_name={task_name}")
            return response.state_execute == 0
        except rospy.ServiceException as e:
            rospy.logerr(f"Service srv_set_task call failed: {e}")
            return None

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    robot_interface = RobotNavigationInterface(is_init=True)
    # print(robot_interface.robot_add_task(cmd_attr='New', task_attr='3', task_id=0, task_name='1楼侯梯点'))
    print(robot_interface.robot_add_task(cmd_attr='Cancel', task_attr='', task_id=0, task_name=''))
    # print(robot_interface.robot_get_task_point_name(target_point=2))
    print(robot_interface.get_robot_position())