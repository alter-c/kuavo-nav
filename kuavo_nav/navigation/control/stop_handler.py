import rospy
import threading
from std_msgs.msg import Empty


class StopHandler:
    """中断管理模块"""
    def __init__(self, _stop_topic="/navigation_stop"):
        self._stop_topic = _stop_topic
        self._stop_event = threading.Event()
        
        rospy.loginfo(f"StopHandler: Subscribed to {self._stop_topic}")
    
    def request_stop(self):
        """发布停止信号"""
        self._stop_event.set()
    
    def is_stop(self):
        """检查停止信号"""
        return self._stop_event.is_set()
    
    def clear_stop(self):
        """清除停止信号"""
        self._stop_event.clear()