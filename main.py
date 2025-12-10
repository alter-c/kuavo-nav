#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from kuavo_nav.navigation import PoseProvider
from kuavo_nav import NavigationCore, NavigationManager


# TODO 添加目标节点消息发布(SLAM坐标->ROS消息)
# TODO 定义任务节点采集与读取

if __name__ == '__main__':
    rospy.init_node('kuavo_navigation', anonymous=True)

    # 初始化子模块及导航节点(默认订阅位姿话题/robot_pose)
    pose_provider = PoseProvider(pose_topic="/pose")  # 里程计模拟slam位姿 
    navigator = NavigationCore(pose_provider=pose_provider)
    nav_manager = NavigationManager(navigator=navigator)

    nav_manager.start()
    rospy.loginfo("Kuavo Navigation Node is running...")

    # 异常退出处理
    def nav_shutdown():
        navigator.shutdown()
    rospy.on_shutdown(nav_shutdown)

    rospy.spin()

