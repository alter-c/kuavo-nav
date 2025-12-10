import rospy
import json
import threading
import uuid
from datetime import datetime
from std_msgs.msg import String

from .navigation.core import NavigationCore

# TODO 优化命令格式
# TODO 同步core优化导航状态
class NavigationManager:
    """导航管理模块"""
    def __init__(
        self,
        navigator: NavigationCore=None,
        rate_hz: int=10
    ):
        if navigator is None:
            raise ValueError("NavigationCore is not initialized")
        
        self._navigator = navigator
        self._rate = rospy.Rate(rate_hz)
        self._lock = threading.Lock()

        # 外部请求消息处理
        self.cmd_sub = None

        # 任务相关信息
        self.task_id = None
        self.cat_id = None
        self.start_time = None

        # 导航相关状态发布
        self.state_pub = None
        self.state_thread = None

    def start(self):
        self.cmd_sub = rospy.Subscriber('/navigation/command', String, self._cmd_callback)
        self.state_pub = rospy.Publisher("/navigation/state", String, queue_size=10)
        self.state_thread = threading.Thread(target=self._state_pub_loop, daemon=True)
        self.state_thread.start()
        rospy.loginfo("Navigation manager started!")

    def _state_pub_loop(self):
        """持续更新并发布导航状态"""
        while not rospy.is_shutdown():
            with self._lock:
                task_id = self.task_id
                cat_id = self.cat_id
                start_time = self.start_time
            pose = self._navigator._pose_provider.get_pose()
            dist_user = None 
            dist_prod = None # TODO 计算距离
            status = self._navigator.is_running
            nav_time = round((rospy.Time.now() - start_time).to_sec()) if start_time else None
            update_time = datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ")

            state_msg = {
                "task_id": task_id,
                "category_id": cat_id,
                "status": status,
                "distance_user": dist_user,
                "distance_prod": dist_prod,
                "navigation_time": nav_time,
                "update_time": update_time
            }
            self.state_pub.publish(String(data=json.dumps(state_msg)))
            self._rate.sleep()

    def _cmd_callback(self, msg: String):
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

        self._update_task(cid=None)
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
    
    def _update_task(self, cid: str=None):
        """更新任务信息"""
        with self._lock:
            self.task_id = str(uuid.uuid4())
            self.cat_id = cid
            self.start_time = rospy.Time.now()