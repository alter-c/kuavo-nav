#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import signal
import math
import sys
import threading
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

from src.navigation.control.base_controller import BaseController
from src.navigation.control.stop_manager import StopManager
from src.navigation.obstacle.obstacle_manager import ObstacleManager

class PoseProvider:
    """位姿数据源模块，带位姿连续性检查"""
    def __init__(self, pose_topic="/robot_pose", max_pos_jump=1.0, max_angle_jump=1.0):
        self._pose_topic = pose_topic
        self._current_pose = (0.0, 0.0, 0.0)
        self._last_pose = (0.0, 0.0, 0.0)
        self._lock = threading.Lock()
        self._pose_received = False
        self._max_pos_jump = max_pos_jump      # 最大位置跳变阈值（米）
        self._max_angle_jump = max_angle_jump  # 最大角度跳变阈值（弧度）
        self._last_timestamp = rospy.Time(0)
        
        # 订阅位姿话题
        self._pose_sub = rospy.Subscriber(
            self._pose_topic, 
            PoseWithCovarianceStamped, 
            self._pose_callback
        )
        
        rospy.loginfo(f"PoseProvider: Subscribed to {self._pose_topic}")
    
    def _pose_callback(self, msg):
        """位姿回调函数，检查位姿连续性"""
        current_time = rospy.Time.now()
        time_diff = (current_time - msg.header.stamp).to_sec()
        
        # 检查消息是否过期（超过500ms）
        if time_diff > 0.5:
            rospy.logwarn(f"Received outdated pose (delay: {time_diff:.3f}s), skipping")
            return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 从四元数转换为欧拉角（仅获取yaw）
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        # 检查位姿跳变（仅在已接收过位姿后）
        if self._pose_received:
            pos_jump = math.hypot(x - self._last_pose[0], y - self._last_pose[1])
            angle_jump = abs(self._normalize_angle(yaw - self._last_pose[2]))
            
            if pos_jump > self._max_pos_jump:
                rospy.logwarn(f"Large position jump detected: {pos_jump:.3f}m (threshold: {self._max_pos_jump}m)")
            
            if angle_jump > self._max_angle_jump:
                rospy.logwarn(f"Large angle jump detected: {angle_jump:.3f}rad (threshold: {self._max_angle_jump}rad)")
        
        with self._lock:
            self._current_pose = (x, y, yaw)
            self._last_pose = (x, y, yaw)
            self._pose_received = True
            self._last_timestamp = msg.header.stamp
    
    def get_current_pose(self):
        """线程安全地获取当前位姿 (x, y, yaw)"""
        with self._lock:
            return self._current_pose
    
    def _normalize_angle(self, angle):
        """将角度归一化到 [-π, π] 范围"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def wait_for_pose(self, timeout=5.0):
        """阻塞直到接收到第一帧位姿或超时（秒）"""
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self._lock:
                if self._pose_received:
                    return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()


class NavigationCore:
    """导航核心模块，三阶段导航：转向目标点 -> 移动到目标点 -> 转向目标朝向"""
    def __init__(
        self,
        pose_provider: PoseProvider,
        vel_publisher: BaseController,
        stop_manager: StopManager,
        obstacle_checker: ObstacleManager = None,
        pos_tolerance=0.1,
        angle_tolerance=0.05,
        rate_hz=10  # 10Hz与SLAM频率匹配
    ):
        self._pose_provider = pose_provider
        self._vel_publisher = vel_publisher
        self._stop_manager = stop_manager
        self._obstacle_checker = obstacle_checker or ObstacleManager()
        
        # 导航参数
        self._pos_tolerance = pos_tolerance
        self._angle_tolerance = angle_tolerance
        self._rate = rospy.Rate(rate_hz)
        
        # 状态管理
        self._is_navigating = False
        self._should_stop = False
        
        rospy.loginfo("NavigationCore: Initialized")

    def _already_at_goal(self, x_target, y_target, yaw_target):
        """Check if robot is already close enough to the target pose."""
        current_pose = self._pose_provider.get_current_pose()
        if current_pose is None:
            return False

        cx, cy, cyaw = current_pose
        gx, gy, gyaw = x_target, y_target, yaw_target

        # position error
        dist = math.hypot(gx - cx, gy - cy)

        # yaw error (wrapped)
        yaw_err = self._normalize_angle(gyaw - cyaw)

        return dist < self._pos_tolerance and abs(yaw_err) < self._angle_tolerance

    
    def start(self, x_target, y_target, yaw_target):
        """启动导航到目标位姿"""
        if self._is_navigating:
            rospy.logwarn("Navigation already running!")
            return False
        
        if not self._pose_provider.wait_for_pose(timeout=5.0):
            rospy.logerr("No pose received within timeout, aborting navigation")
            return False
            
        self._is_navigating = True
        self._should_stop = False
        self._stop_manager.clear_stop()
        
        try:
            rospy.loginfo(f"Starting navigation to ({x_target}, {y_target}, {yaw_target})")

            if self._already_at_goal(x_target, y_target, yaw_target):
                rospy.loginfo("Already at target pose, skipping navigation")
                return True
            
            # 阶段1: 转向目标点
            self._rate.sleep()
            if not self._rotate_to_direction(x_target, y_target):
                rospy.logwarn("Navigation interrupted during rotation to direction")
                return False
            
            # 阶段2: 移动到目标点
            self._rate.sleep()
            if not self._move_to_position(x_target, y_target):
                rospy.logwarn("Navigation interrupted during movement to position")
                return False
            
            # 阶段3: 转向目标朝向
            self._rate.sleep()
            if not self._rotate_to_yaw(yaw_target):
                rospy.logwarn("Navigation interrupted during rotation to yaw")
                return False
            
            rospy.loginfo("Navigation completed successfully")
            return True
            
        except Exception as e:
            rospy.logerr(f"Navigation error: {e}")
            return False
        finally:
            # 确保停止
            self._vel_publisher.reset()
            self._is_navigating = False
    
    def stop(self):
        """请求停止导航"""
        self._stop_manager.request_stop()
    
    def is_navigating(self):
        """检查是否在运行"""
        return self._is_navigating
    
    def _check_stop_request(self):
        """检查停止请求"""
        if self._stop_manager.is_stop():
            self._should_stop = True
            return True
        return False
    
    def _rotate_to_direction(self, x_target, y_target):
        """阶段1: 转向目标点方向"""
        current_x, current_y, current_yaw = self._pose_provider.get_current_pose()
        target_yaw = math.atan2(y_target - current_y, x_target - current_x)
        return self._rotate_to(target_yaw)
    
    def _rotate_to_yaw(self, target_yaw):
        """阶段3: 转向目标朝向"""
        return self._rotate_to(target_yaw)
    
    def _rotate_to(self, target_yaw):
        """原地旋转到目标角度"""
        while not rospy.is_shutdown():
            if self._check_stop_request():
                return False
            
            _, _, current_yaw = self._pose_provider.get_current_pose()
            
            # 计算角度差（归一化到 [-π, π]）
            angle_diff = self._normalize_angle(target_yaw - current_yaw)
            
            if abs(angle_diff) < self._angle_tolerance:
                rospy.loginfo(f"Rotation completed. Current: {current_yaw}, Target: {target_yaw}")
                break
            
            # P控制器
            angular_speed = 1.0 * angle_diff
            self._vel_publisher.move(0.0, angular_speed)
            self._rate.sleep()
        
        # 停止旋转
        self._vel_publisher.reset()
        return True
    
    def _move_to_position(self, x_target, y_target):
        """阶段2: 直线移动到目标位置"""
        while not rospy.is_shutdown():
            if self._check_stop_request():
                return False
            
            # 预留避障检查点
            if self._obstacle_checker.check():
                rospy.logwarn("Obstacle detected! (Placeholder)")
                pass
            
            current_x, current_y, _ = self._pose_provider.get_current_pose()
            
            # 计算到目标的距离
            dx = x_target - current_x
            dy = y_target - current_y
            distance = math.hypot(dx, dy)
            
            if distance < self._pos_tolerance:
                rospy.loginfo(f"Movement completed. Distance: {distance}")
                break
            
            # 计算前进方向的角度
            target_angle = math.atan2(dy, dx)
            current_x, current_y, current_yaw = self._pose_provider.get_current_pose()
            
            # 计算朝向误差（用于微调）
            angle_error = self._normalize_angle(target_angle - current_yaw)
            
            # P控制器
            linear_speed = 0.5 * distance
            angular_speed = 1.0 * angle_error
            
            self._vel_publisher.move(linear_speed, angular_speed)
            self._rate.sleep()
        
        # 停止移动
        self._vel_publisher.reset()
        return True
    
    def _normalize_angle(self, angle):
        """将角度归一化到 [-π, π] 范围"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    """主函数 - 演示如何使用导航模块"""
    rospy.init_node('modular_navigation', anonymous=True)
    
    # 初始化各模块
    pose_provider = PoseProvider(pose_topic="/pose", max_pos_jump=1.0, max_angle_jump=1.0)
    vel_publisher = BaseController(cmd_vel_topic="/cmd_vel")
    stop_manager = StopManager()
    obstacle_checker = ObstacleManager()
    
    # 创建导航核心
    navigator = NavigationCore(
        pose_provider=pose_provider,
        vel_publisher=vel_publisher,
        stop_manager=stop_manager,
        obstacle_checker=obstacle_checker,
        rate_hz=10  # 10Hz频率
    )
    
    # 示例：导航到目标点 (1.0, 1.0, 0.0)
    target_x = 0.0
    target_y = 0.0
    target_yaw = 0.0
    
    rospy.loginfo("Press Enter to start navigation...")
    input()
    
    success = navigator.start(target_x, target_y, target_yaw)
    
    if success:
        rospy.loginfo("Navigation succeeded!")
    else:
        rospy.logwarn("Navigation failed or was interrupted!")
    
    # rospy.spin()

def signal_handler(sig, frame):
    """设置ctrl+c信号处理器"""
    rospy.loginfo("按下Ctrl+C, 正在退出程序...")
    rospy.signal_shutdown("User interrupt")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()



