#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import signal
import sys
import math

from kuavo_nav.navigation import PoseProvider
from kuavo_nav import NavigationCore


def signal_handler(sig, frame):
    rospy.loginfo("Shutting down navigation node...")
    rospy.signal_shutdown("User interrupt")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# TODO 添加目标节点消息发布(SLAM坐标->ROS消息)
# TODO 定义任务节点采集与读取

def main():
    """启动导航模块"""
    rospy.init_node('kuavo_navigation', anonymous=True)
    
    # 初始化子模块及导航节点
    pose_provider = PoseProvider(pose_topic="/pose")

    navigator = NavigationCore(
        pose_provider=pose_provider
    )
    
    # 目标点导航示例(全局坐标)
    target_x = 1.0
    target_y = 1.0
    target_yaw = math.pi / 2 
    
    rospy.loginfo("Press Enter to start navigation...")
    input()
    
    try:
        navigator.start(target_x, target_y, target_yaw)
        # navigator.stop()
        rospy.loginfo("Navigation succeeded!")
    except Exception as e:
        rospy.logwarn(f"Navigation failed: {e}")

    # rospy.spin()

if __name__ == '__main__':
    main()



