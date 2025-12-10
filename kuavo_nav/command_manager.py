import rospy
import json
import threading
from std_msgs.msg import String

from .navigation.core import NavigationCore

# TODO 优化命令格式
class CommandManager:
    def __init__(
        self,
        navigator: NavigationCore=None
    ):
        if navigator is None:
            raise ValueError("NavigationCore is not initialized")
        
        self._navigator = navigator

        # 外部请求消息处理
        self._cmd_sub = rospy.Subscriber('/navigation/command', String, self._command_callback)

    def _command_callback(self, msg: String):
        """解析命令并执行"""
        try:
            cmd_dict = json.loads(msg.data)
            command = cmd_dict.get("command", "")
            if command == "navigate":
                rospy.loginfo("Received navigate command")
                self._handle_navigate(cmd_dict)
            elif command == "cancel":
                rospy.loginfo("Received cancel command")
                self._handle_cancel()
            else:
                rospy.logwarn(f"Unknown command: {command}")
                
        except Exception as e:
            rospy.logerr(f"Unexpected error during command process: {e}")

    def _handle_navigate(self, cmd_dict: dict):
        """处理导航命令"""
        target = cmd_dict.get("target", {})
        x = target.get("x", 0.0)
        y = target.get("y", 0.0)
        yaw = target.get("yaw", 0.0)

        # 线程启动导航
        nav_thread = threading.Thread(
            target=self._navigator.navigate, 
            args=(x, y, yaw),
            daemon=True
        )
        nav_thread.start()
        return
    
    def _handle_cancel(self):
        """处理取消导航命令"""
        self._navigator.cancel()
        return