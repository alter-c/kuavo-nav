import rospy
import json
import uuid
import math
import threading
from datetime import datetime
from std_msgs.msg import String

from .navigation.core import NavigationCore
from .action import ActionExecutor


class NavigationManager:
    """导航管理模块"""
    def __init__(
        self,
        navigator: NavigationCore=None,
        actor: ActionExecutor=None,
        map_file: str="configs/task_map.json",
        rate_hz: int=10
    ):
        if navigator is None:
            raise ValueError("NavigationCore is not initialized")
        
        self._navigator = navigator
        self._actor = actor or ActionExecutor()
        self._map = load_map(map_file)
        self._rate = rospy.Rate(rate_hz)
        self._lock = threading.Lock()

        # 外部请求消息处理
        self.cmd_sub = None

        # 任务相关信息
        self.task_id = None
        self.cat_id = None
        self.start_time = None
        self.target_pose = None

        # 导航相关状态发布
        self.state_pub = None
        self.state_thread = None

    def start(self):
        self.cmd_sub = rospy.Subscriber('/navigation/command', String, self._cmd_callback)
        self.state_pub = rospy.Publisher("/navigation/state", String, queue_size=10)
        self.state_thread = threading.Thread(target=self._state_pub_loop, daemon=True)
        self.state_thread.start()
        rospy.loginfo("Navigation manager started!")

    def get_state(self):
        """获取导航状态信息"""
        with self._lock:
            task_id = self.task_id
            cat_id = self.cat_id
            start_time = self.start_time
            target_pose = self.target_pose
        pose = self._navigator._pose_provider.pose
        status = self._navigator.status
        dist_user = None 
        dist_prod = math.hypot(
            pose[0] - target_pose[0], 
            pose[1] - target_pose[1]
        ) if pose and target_pose else None
        nav_time = round(
            (rospy.Time.now() - start_time).to_sec()
        ) if start_time else None
        update_time = datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ")

        return {
            "task_id": task_id,
            "category_id": cat_id,
            "status": status,
            "distance_user": dist_user,
            "distance_prod": dist_prod,
            "navigation_time": nav_time,
            "update_time": update_time
        }

    def _state_pub_loop(self):
        """持续更新并发布导航状态"""
        while not rospy.is_shutdown():
            state = self.get_state()
            self.state_pub.publish(String(data=json.dumps(state)))
            # rospy.loginfo(f"Published /navigation/state: {state}")
            self._rate.sleep()

    def _cmd_callback(self, msg: String):
        """解析命令并执行"""
        try:
            cmd_dict = json.loads(msg.data)
            command = cmd_dict.get("command")
            if command == "navigate":
                rospy.loginfo("Received navigate command")
                self._handle_navigate(cmd_dict)
            elif command == "cancel":
                rospy.loginfo("Received cancel command")
                self._handle_cancel()
            elif command == "action":
                rospy.loginfo("Received action command")
                self._handle_action(cmd_dict)
            elif command == "gesture":
                rospy.loginfo("Received gesture command")
                self._handle_gesture(cmd_dict)
            else:
                rospy.logwarn(f"Unknown command: {command}")
                
        except Exception as e:
            rospy.logerr(f"Error during command process: {e}")

    def _handle_navigate(self, cmd_dict: dict):
        """处理导航命令"""
        try:
            target = cmd_dict.get("target")
            cid, pose = self._parse_target(target)
        except Exception as e:
            rospy.logerr(f"Error during navigation target parsing: {e}")
            return
        self._update_task(cid=cid, pose=pose)
        # 线程启动导航
        x, y, yaw = pose
        # TODO 封装导航和动作, 统一线程执行
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
    
    def _handle_action(self, cmd_dict: dict):
        """处理动作命令"""
        action = cmd_dict.get("target")
        self._actor.execute(action)
        # action_thread = threading.Thread(
        #     target=self._actor.execute,
        #     args=(action,),
        #     daemon=True
        # )
        # action_thread.start()
        return
    
    def _handle_gesture(self, cmd_dict: dict):
        """处理对话手势命令"""
        gesture = cmd_dict.get("target")
        if gesture == "start":
            if self._navigator.status == "arrived" or self._navigator.status == "idle":
                gesture_thread = threading.Thread(
                    target=self._actor.gesture,
                    daemon=True
                )
                gesture_thread.start()
            else:
                rospy.logwarn("Not arrived, cannot execute gesture")
                return
        elif gesture == "stop":
            self._actor.stop_gesture()
        return
    
    def _parse_target(self, target):
        """解析导航目标"""
        # 任务点导航
        if isinstance(target, str):
            if self._map is None or target not in self._map:
                raise ValueError(f"Target category id '{target}' not found in map")
            cid_info = self._map[target]
            if not cid_info.get("pose"):
                raise ValueError(f"Category id '{target}' has no pose information")
            cid = target
            x, y, yaw = cid_info.get("pose")

        # 坐标导航
        elif isinstance(target, dict):
            cid = None
            x = target.get("x")
            y = target.get("y")
            yaw = target.get("yaw")

        else:
            raise TypeError(f"Invalid target format: {type(target)}")
        
        return cid, (x, y, yaw)

    def _update_task(self, cid: str=None, pose: tuple=None):
        """更新任务信息"""
        with self._lock:
            self.task_id = str(uuid.uuid4())
            self.cat_id = cid
            self.start_time = rospy.Time.now()
            self.target_pose = pose

    
def load_map(file_path: str) -> dict:
    try:
        with open(file_path, 'r') as f:
            task_map = json.load(f)
    except Exception as e:
        print(f"Failed to load task map: {e}")
        task_map = None
    return task_map
