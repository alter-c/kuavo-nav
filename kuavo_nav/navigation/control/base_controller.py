import rospy
from geometry_msgs.msg import Twist

class BaseController:
    """基础运动控制模块"""
    def __init__(self, cmd_vel_topic="/cmd_vel", max_linear=0.4, max_angular=0.5):
        self._cmd_vel_topic = cmd_vel_topic
        self._max_linear = max_linear
        self._max_angular = max_angular
        
        self._cmd_vel_pub = rospy.Publisher(
            self._cmd_vel_topic, 
            Twist, 
            queue_size=10
        )
        
        rospy.loginfo(f"BaseController: Publishing to {self._cmd_vel_topic}")
    
    def _pub_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        """发布基础速度命令"""
        # 限制最大速度
        linear_x = max(-self._max_linear, min(self._max_linear, linear_x))
        angular_z = max(-self._max_angular, min(self._max_angular, angular_z))
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_z
        
        self._cmd_vel_pub.publish(cmd)

    def move(self, linear_x=0.0, angular_z=0.0):
        self._pub_cmd_vel(linear_x, angular_z)

    def reset(self):
        self._pub_cmd_vel(0.0, 0.0)