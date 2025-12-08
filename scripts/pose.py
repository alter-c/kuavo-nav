#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
模拟SLAM位姿发布器
用于在没有真实SLAM的情况下测试导航模块
"""

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_from_euler


class MockSLAMPosePublisher:
    """
    模拟SLAM位姿发布器
    订阅/cmd_vel，基于速度积分计算位姿，发布到/robot_pose
    """
    def __init__(self):
        # 初始化节点
        rospy.init_node('mock_slam_pose_publisher', anonymous=True)
        
        # 机器人状态（初始位姿）
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # 朝向角度（弧度）
        
        # 速度状态
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        # 时间戳
        self.last_time = rospy.Time.now()
        
        # 订阅/cmd_vel
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel', 
            Twist, 
            self.cmd_vel_callback
        )
        
        # 发布/robot_pose（模拟SLAM输出）
        self.pose_pub = rospy.Publisher(
            '/robot_pose', 
            PoseStamped, 
            queue_size=10
        )
        
        # 发布频率（10Hz，模拟SLAM频率）
        self.rate = rospy.Rate(100)
        
        rospy.loginfo("MockSLAMPosePublisher: Initialized")
    
    def cmd_vel_callback(self, msg):
        """速度命令回调"""
        self.vx = msg.linear.x
        self.vy = 0.0  # 假设机器人只能前后移动
        self.vtheta = msg.angular.z
    
    def update_pose(self):
        """基于速度积分更新位姿"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt > 0:
            # 简单的积分更新（欧拉积分）
            dx_world = self.vx * math.cos(self.theta) * dt - self.vy * math.sin(self.theta) * dt
            dy_world = self.vx * math.sin(self.theta) * dt + self.vy * math.cos(self.theta) * dt

            self.x += dx_world
            self.y += dy_world
            self.theta += self.vtheta * dt

            
            # 限制角度范围到 [-π, π]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.last_time = current_time
    
    def publish_pose(self):
        """发布位姿消息"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"  # 假设使用map坐标系
        
        # 设置位置
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        
        # 设置朝向（从欧拉角转换为四元数）
        quat = quaternion_from_euler(0, 0, self.theta)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
    
    def run(self):
        """主循环"""
        rospy.loginfo("MockSLAMPosePublisher: Starting...")
        
        while not rospy.is_shutdown():
            # 更新位姿
            self.update_pose()
            
            # 发布位姿
            self.publish_pose()
            print(self.x, self.y, self.theta)
            
            # 等待下一个周期
            self.rate.sleep()


def main():
    """主函数"""
    try:
        publisher = MockSLAMPosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("MockSLAMPosePublisher: Shutting down...")


if __name__ == '__main__':
    main()



