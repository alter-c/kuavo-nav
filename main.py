#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from kuavo_nav.navigation import PoseProvider
from kuavo_nav import NavigationCore, NavigationManager


if __name__ == '__main__':
    rospy.init_node('kuavo_navigation', anonymous=True)

    # 初始化导航节点
    navigator = NavigationCore()
    nav_manager = NavigationManager(navigator=navigator)

    nav_manager.start()
    rospy.loginfo("Kuavo Navigation Node is running...")

    # 异常退出处理
    def nav_shutdown():
        navigator.shutdown()
    rospy.on_shutdown(nav_shutdown)

    rospy.spin()

