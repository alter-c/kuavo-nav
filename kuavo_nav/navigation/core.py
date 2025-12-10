import rospy
import math
from enum import Enum
from std_msgs.msg import String

from . import BaseController, StopHandler, ObstacleDetector, PoseProvider


class NavigationState(Enum):
    IDLE = "idle"
    STARTING = "starting"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    OBSTACLE = "obstacle"
    FAILED = "failed"

# TODO 当前导航状态管理, 并定义相关类持续管理和发布
class NavigationCore:
    """
    导航核心模块
    三阶段导航：转向目标点 -> 移动到目标点 -> 转向目标朝向
    """
    def __init__(
        self,
        pose_provider: PoseProvider=None,   
        vel_publisher: BaseController=None,
        stop_handler: StopHandler=None,
        obstacle_detector: ObstacleDetector=None,
        pos_tolerance: float=0.1,
        angle_tolerance: float=0.05,
        rate_hz: int=10
    ):
        self._pose_provider = pose_provider or PoseProvider()
        self._vel_publisher = vel_publisher or BaseController()
        self._stop_handler = stop_handler or StopHandler()
        self._obstacle_detector = obstacle_detector or ObstacleDetector()

        # 导航容差及频率参数
        self._pos_tolerance = pos_tolerance
        self._angle_tolerance = angle_tolerance
        self._rate = rospy.Rate(rate_hz)
        
        # 状态管理
        self.is_running = False
        
        rospy.loginfo("Navigation core initialized!")

    
    def navigate(self, x_target, y_target, yaw_target):
        """启动导航到目标位姿"""
        if self.is_running:
            rospy.logwarn("Navigation already running!")
            return False
        
        if not self._pose_provider.wait_for_pose():
            rospy.logerr("Timed out waiting for pose, aborting navigation")
            return False
            
        self.is_running = True
        self._stop_handler.clear_stop()
        
        try:
            rospy.loginfo(f"Starting navigation to target (x={x_target:.3f}m, y={y_target:.3f}m, yaw={yaw_target:.3f}rad)")

            if self._at_target(x_target, y_target, yaw_target):
                rospy.loginfo("Already at target pose, skipping navigation")
                return True
            
            # 阶段1: 转向目标点
            if not self._rotate_to_direction(x_target, y_target):
                rospy.logwarn("Navigation interrupted during stage1: Rotation to direction")
                return False
            
            # 阶段2: 移动到目标点
            if not self._move_to_position(x_target, y_target):
                rospy.logwarn("Navigation interrupted during stage2: Movement to position")
                return False
            
            # 阶段3: 转向目标朝向
            if not self._rotate_to_yaw(yaw_target):
                rospy.logwarn("Navigation interrupted during stage3: Rotation to yaw")
                return False
            
            rospy.loginfo("Navigation completed!")
            return True
            
        except Exception as e:
            rospy.logerr(f"Navigation error: {e}")
            return False
        
        finally:
            # 确保停止
            self.is_running = False
            self._vel_publisher.reset()
    
    def cancel(self):
        """停止导航"""
        self._stop_handler.request_stop()
        rospy.loginfo("Navigation cancel requested")
    

    def _at_target(self, x_target, y_target, yaw_target):
        """检查是否已经到达目标位姿, 避免不必要导航"""
        current_pose = self._pose_provider.get_pose()
        if current_pose is None:
            return False

        cx, cy, cyaw = current_pose
        tx, ty, tyaw = x_target, y_target, yaw_target

        # position and yaw error
        dist = math.hypot(tx - cx, ty - cy)
        yaw_err = self._normalize_angle(tyaw - cyaw)

        return dist < self._pos_tolerance and abs(yaw_err) < self._angle_tolerance
    
    def _rotate_to(self, target_yaw):
        """原地旋转到目标角度"""
        success = False
        while not rospy.is_shutdown():
            if self._stop_handler.is_stop():
                break
            
            _, _, current_yaw = self._pose_provider.get_pose()
            
            # 判断是否到达目标角度
            angle_error = self._normalize_angle(target_yaw - current_yaw)
            if abs(angle_error) < self._angle_tolerance:
                rospy.loginfo(f"Rotation completed. Current: {current_yaw:.3f}rad, Target: {target_yaw:.3f}rad")
                success = True
                break
            
            # P控制器
            cmd_angular = 1.0 * angle_error
            self._vel_publisher.move(0.0, cmd_angular)
            self._rate.sleep()
        
        # 停止旋转
        self._vel_publisher.reset()
        self._rate.sleep()
        return success
    
    def _rotate_to_direction(self, x_target, y_target):
        """阶段1: 转向目标点方向"""
        current_x, current_y, _ = self._pose_provider.get_pose()
        target_yaw = math.atan2(y_target - current_y, x_target - current_x)
        rospy.loginfo(f"Rotating to target position (direction yaw={target_yaw:.3f}rad)")
        return self._rotate_to(target_yaw)
    
    def _move_to_position(self, x_target, y_target):
        """阶段2: 直线微调移动到目标位置"""
        success = False
        rospy.loginfo(f"Moving to target position (x={x_target:.3f}m, y={y_target:.3f}m)")
        while not rospy.is_shutdown():
            if self._stop_handler.is_stop():
                break
            
            # TODO 避障
            if self._obstacle_detector.check():
                rospy.logwarn("Obstacle detected!")
                pass
            
            current_x, current_y, current_yaw = self._pose_provider.get_pose()
            
            # 计算到目标的距离
            dx = x_target - current_x
            dy = y_target - current_y
            distance = math.hypot(dx, dy)
            
            if distance < self._pos_tolerance:
                rospy.loginfo(f"Movement completed. Distance: {distance}")
                success = True
                break
            
            # 微调朝向误差
            desired_yaw = math.atan2(dy, dx)
            yaw_error = self._normalize_angle(desired_yaw - current_yaw)
            
            # P控制器
            cmd_linear = 0.5 * distance
            cmd_angular = 1.0 * yaw_error
            self._vel_publisher.move(cmd_linear, cmd_angular)
            self._rate.sleep()
        
        # 停止移动
        self._vel_publisher.reset()
        self._rate.sleep()
        return success
    
    def _rotate_to_yaw(self, target_yaw):
        """阶段3: 转向目标朝向"""
        rospy.loginfo(f"Rotating to target yaw={target_yaw:.3f}rad")
        return self._rotate_to(target_yaw)
    
    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
