import rospy
import math
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class PoseProvider:
    """位姿数据管理模块"""
    def __init__(self, pose_topic="/robot_pose", max_pos_jump=0.5, max_angle_jump=0.5):
        self._pose_topic = pose_topic
        self._lock = threading.Lock()
        self._pose = None
        self._stamp = rospy.Time(0)
        self._max_pos_jump = max_pos_jump      # 最大位置跳变阈值
        self._max_angle_jump = max_angle_jump  # 最大角度跳变阈值
        
        # 位姿消息类型固定 PoseWithCovarianceStamped
        self._pose_sub = rospy.Subscriber(
            self._pose_topic, 
            PoseWithCovarianceStamped, 
            self._pose_callback
        )
        
        rospy.loginfo(f"PoseProvider: Subscribed to {self._pose_topic}")

    @property
    def pose(self):
        """获取当前位姿"""
        with self._lock:
            return self._pose
    
    def _pose_callback(self, msg):
        """位姿信息回调"""
        if msg is None:
            rospy.logerr("Received None pose message!")
            return
        
        # 检查消息是否过期
        delay = (rospy.Time.now() - msg.header.stamp).to_sec()
        if delay > 0.5:
            rospy.logwarn(f"Received outdated pose (delay: {delay:.3f}s), skipping")
            return
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        # 四元数转欧拉角
        _, _, yaw = euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        yaw = self._normalize_angle(yaw)
        
        # 检查位姿跳变并更新 
        # TODO 位姿滤波
        with self._lock:
            if self._pose is not None:
                last_x, last_y, last_yaw = self._pose
                pos_jump = math.hypot(x - last_x, y - last_y)
                angle_jump = abs(self._normalize_angle(yaw - last_yaw))
                
                if pos_jump > self._max_pos_jump:
                    rospy.logwarn(f"Large position jump detected: {pos_jump:.3f}m")
                if angle_jump > self._max_angle_jump:
                    rospy.logwarn(f"Large angle jump detected: {angle_jump:.3f}rad")

            self._pose = (x, y, yaw)
            self._stamp = msg.header.stamp
    
    def wait_for_pose(self, timeout=5.0):
        """阻塞等待位姿消息"""
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._pose is not None:
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()
        return False
    
    def _normalize_angle(self, angle):
        """将角度归一化到 [-π, π] 范围"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle