import rospy
import math
import threading
from enum import Enum
from std_msgs.msg import String

from . import BaseController, StopHandler, ObstacleDetector, PoseProvider, RoutePlanner


class NavStatus(Enum):
    IDLE = "idle"
    STARTING = "starting"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    OBSTACLE = "obstacle"
    FAILED = "failed"

class NavigationCore:
    """
    导航核心模块
    导航逻辑：(转向目标点 -> 移动到目标点)*n -> 转向目标朝向
    """
    def __init__(
        self,
        route_file: str="configs/route_map.json",
        pose_provider: PoseProvider=None,   
        vel_publisher: BaseController=None,
        stop_handler: StopHandler=None,
        obstacle_detector: ObstacleDetector=None,
        pos_tolerance: float=0.2,
        angle_tolerance: float=0.05,
        rate_hz: int=10
    ):
        self._route_planner = RoutePlanner(route_file)
        self._pose_provider = pose_provider or PoseProvider()
        self._vel_publisher = vel_publisher or BaseController()
        self._stop_handler = stop_handler or StopHandler()
        self._obstacle_detector = obstacle_detector or ObstacleDetector()

        # 导航容差及频率参数
        self._pos_tolerance = pos_tolerance
        self._angle_tolerance = angle_tolerance
        self._rate = rospy.Rate(rate_hz)
        
        # 状态管理
        self._status_lock = threading.Lock()
        self._status = NavStatus.IDLE
        self._run_lock = threading.Lock()
        self._running = False
        
        rospy.loginfo("Navigation core initialized!")

    @property
    def status(self):
        """获取当前导航状态"""
        with self._status_lock:
            return self._status.value
    
    def navigate(self, x_target, y_target, yaw_target):
        """启动导航到目标位姿"""
        with self._run_lock:
            if self._running:
                rospy.logwarn("Navigation already running!")
                return False
            else:
                rospy.loginfo("Navigation starting...")
                self._running = True
                self._update_status(NavStatus.STARTING)
        self._stop_handler.clear_stop()
        
        try:
            if not self._pose_provider.wait_for_pose():
                rospy.logerr("Timed out waiting for pose, aborting navigation")
                self._update_status(NavStatus.FAILED)
                return False
            
            if self._at_target(x_target, y_target, yaw_target):
                rospy.loginfo("Already at target pose, skipping navigation")
                self._update_status(NavStatus.ARRIVED)
                return True
            
            route_list = self._route_planner.plan(
                start = self._pose_provider.pose,
                end = (x_target, y_target, yaw_target)
            )
            if not route_list:
                rospy.logerr("No valid route found, aborting navigation")
                self._update_status(NavStatus.FAILED)
                return False

            rospy.loginfo(f"Navigating to target (x={x_target:.3f}m, y={y_target:.3f}m, yaw={yaw_target:.3f}rad)")
            self._update_status(NavStatus.NAVIGATING)

            for i, point in enumerate(route_list):
                # rospy.loginfo(f"Route point: x={point[0]:.3f}, y={point[1]:.3f}, yaw={point[2]:.3f}")
                x_point, y_point = point
                
                # 跳过过近的路径点
                current_x, current_y, _ = self._pose_provider.pose
                dx = x_point - current_x
                dy = y_point - current_y
                distance = math.hypot(dx, dy)
                if distance < 0.3:
                    rospy.loginfo(f"Skipping close route point (x={x_point:.3f}, y={y_point:.3f})")
                    continue

                # 阶段1: 转向目标点
                if not self._rotate_to_direction(x_point, y_point):
                    rospy.logwarn("Navigation interrupted during stage1: Rotation to direction")
                    return False
                
                # 阶段2: 移动到目标点
                if not self._move_to_position(x_point, y_point):
                    rospy.logwarn("Navigation interrupted during stage2: Movement to position")
                    return False
            
            # 阶段3: 转向目标朝向
            if not self._rotate_to_yaw(yaw_target):
                rospy.logwarn("Navigation interrupted during stage3: Rotation to yaw")
                return False
            
            rospy.loginfo("Navigation completed!")
            self._update_status(NavStatus.ARRIVED)
            return True
            
        except Exception as e:
            rospy.logerr(f"Navigation error: {e}")
            self._update_status(NavStatus.FAILED)
            return False
        
        finally:
            # 确保停止
            with self._run_lock:
                self._running = False 
            self._vel_publisher.reset()
    
    def cancel(self):
        """停止导航"""
        rospy.loginfo("Navigation cancel requested")
        self._stop_handler.request_stop()
        self._update_status(NavStatus.FAILED)
    
    def shutdown(self):
        """安全退出导航"""
        rospy.logwarn("Navigation core shutdown!")
        self._vel_publisher.reset()

    def _at_target(self, x_target, y_target, yaw_target):
        """检查是否已经到达目标位姿, 避免不必要导航"""
        current_pose = self._pose_provider.pose
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
            
            # TODO 障碍检测
            if self._obstacle_detector.check():
                # 避障失败或中途停止
                if not self._handle_obstacle():
                    break

            _, _, current_yaw = self._pose_provider.pose
            
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
        # self._vel_publisher.reset()
        self._rate.sleep()
        return success
    
    def _rotate_to_direction(self, x_target, y_target):
        """阶段1: 转向目标点方向"""
        current_x, current_y, current_yaw = self._pose_provider.pose
        target_yaw = math.atan2(y_target - current_y, x_target - current_x)
        print("current_yaw:", current_yaw, "target_yaw:", target_yaw)
        # 如果已经朝向目标方向, 则跳过旋转
        if abs(self._normalize_angle(target_yaw - current_yaw)) < 0.3:
            rospy.loginfo("Already facing target direction, skipping rotation")
            return True
        rospy.loginfo(f"Rotating to target position (direction yaw={target_yaw:.3f}rad)")
        return self._rotate_to(target_yaw)
    
    def _move_to_position(self, x_target, y_target, reset=False):
        """阶段2: 直线微调移动到目标位置"""
        success = False
        rospy.loginfo(f"Moving to target position (x={x_target:.3f}m, y={y_target:.3f}m)")
        while not rospy.is_shutdown():
            if self._stop_handler.is_stop():
                break
            
            if self._obstacle_detector.check():
                if not self._handle_obstacle():
                    break

            current_x, current_y, current_yaw = self._pose_provider.pose
            
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
            # cmd_linear = 0.5 * distance
            cmd_linear = 0.5
            cmd_angular = 1.0 * yaw_error
            self._vel_publisher.move(cmd_linear, cmd_angular)
            self._rate.sleep()
        
        # 停止移动
        # self._vel_publisher.reset()
        self._rate.sleep()
        return success
    
    def _rotate_to_yaw(self, target_yaw):
        """阶段3: 转向目标朝向"""
        rospy.loginfo(f"Rotating to target yaw={target_yaw:.3f}rad")
        return self._rotate_to(target_yaw)
    
    def _handle_obstacle(self):
        """避障处理"""
        # 停障
        rospy.logwarn("Obstacle detected!")
        self._vel_publisher.reset()
        self._update_status(NavStatus.OBSTACLE)
        # TODO 避障语音
        self._obstacle_detector.report()
        while self._obstacle_detector.check():
            if self._stop_handler.is_stop():
                # 障碍物未清除且收到停止指令, 直接退出, 防止死循环
                return False
            self._rate.sleep()
        # 避障后恢复导航状态
        rospy.loginfo("Obstacle cleared")
        self._update_status(NavStatus.NAVIGATING)
        return True

    def _update_status(self, new_status: NavStatus):
        with self._status_lock:
            self._status = new_status
    
    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
